/*
 * props-shim - answer mnld's property reads from a file.
 *
 * mnld reads 16 Android properties at startup and gives up without some of them:
 * "chip is unknown, version 0x0" without persist.vendor.connsys.chipid, and
 * "nvram_init_val not Ready" without vendor.service.nvram_init - both observed
 * on begonia. The environment has no property_service, and writing a real
 * property area (/dev/__properties__, bionic's internal format) would tie this
 * to one Android release, so the shim overrides the four client entry points
 * instead and answers from a plain "name=value" file.
 *
 * Built with no libc of its own (-nostdlib): the only external symbols are the
 * four system calls it needs, which the bionic process already has. That is what
 * makes it safe to preload into an Android binary - there is no second C library
 * in the picture and nothing to mismatch.
 *
 * A name that is not in the file answers empty, which is what an unset property
 * looks like on Android. Nothing is invented.
 */
#define GPS_PROPS_FILE "/data/local/tmp/gps.props"
#define PROP_VALUE_MAX 92
#define MAX_PROPS 64
#define NAME_MAX_LEN 96
#define FILE_MAX 8192

#define NULL ((void *)0)

/*
 * bionic's libc, already loaded in the process. write(2) is what the log part
 * below uses: mnld's own lines go to stderr, because there is no logd in this
 * environment and the kernel log is the only place a trial can read them.
 */
extern int open(const char *path, int flags, ...);
extern long read(int fd, void *buf, unsigned long count);
extern int close(int fd);
extern long write(int fd, const void *buf, unsigned long count);

struct entry {
	char name[NAME_MAX_LEN];
	char value[PROP_VALUE_MAX];
};

static struct entry g_entries[MAX_PROPS];
static int g_count = -1;
static char g_file[FILE_MAX];

static int streq(const char *a, const char *b)
{
	while (*a && *a == *b) {
		a++;
		b++;
	}
	return *a == *b;
}

static void copy(char *dst, const char *src, int max)
{
	int i = 0;

	while (src[i] && i < max - 1) {
		dst[i] = src[i];
		i++;
	}
	dst[i] = '\0';
}

static void load(void)
{
	long n, i = 0;
	int fd;
	struct entry *cur = NULL;
	int field = 0;		/* 0 = name, 1 = value */

	if (g_count >= 0)
		return;
	g_count = 0;

	fd = open(GPS_PROPS_FILE, 0 /* O_RDONLY */);
	if (fd < 0)
		return;
	n = read(fd, g_file, FILE_MAX - 1);
	close(fd);
	if (n <= 0)
		return;
	g_file[n] = '\0';

	while (i < n && g_count < MAX_PROPS) {
		char c = g_file[i];

		if (c == '\n' || c == '\r') {
			if (cur && field == 1)
				g_count++;
			cur = NULL;
			field = 0;
			i++;
			continue;
		}
		if (!cur) {
			if (c == '#') {		/* comment line */
				while (i < n && g_file[i] != '\n')
					i++;
				continue;
			}
			cur = &g_entries[g_count];
			cur->name[0] = '\0';
			cur->value[0] = '\0';
		}
		if (c == '=' && field == 0) {
			field = 1;
			cur->name[sizeof(cur->name) - 1] = '\0';
			i++;
			continue;
		}
		if (field == 0) {
			int len = 0;

			while (cur->name[len])
				len++;
			if (len < NAME_MAX_LEN - 1) {
				cur->name[len] = c;
				cur->name[len + 1] = '\0';
			}
		} else {
			int len = 0;

			while (cur->value[len])
				len++;
			if (len < PROP_VALUE_MAX - 1) {
				cur->value[len] = c;
				cur->value[len + 1] = '\0';
			}
		}
		i++;
	}
	if (cur && field == 1 && g_count < MAX_PROPS)
		g_count++;
}

static const char *lookup(const char *name)
{
	int i;

	load();
	for (i = 0; i < g_count; i++)
		if (streq(g_entries[i].name, name))
			return g_entries[i].value;
	return NULL;
}

int __system_property_get(const char *name, char *value)
{
	const char *v = lookup(name);
	int len = 0;

	if (!v)
		v = "";
	while (v[len] && len < PROP_VALUE_MAX - 1) {
		value[len] = v[len];
		len++;
	}
	value[len] = '\0';
	return len;
}

const void *__system_property_find(const char *name)
{
	int i;

	load();
	for (i = 0; i < g_count; i++)
		if (streq(g_entries[i].name, name))
			return (const void *)&g_entries[i];
	return (const void *)0;
}

int __system_property_read(const void *pi, char *name, char *value)
{
	const struct entry *e = (const struct entry *)pi;
	int len = 0;

	if (!e)
		return 0;
	if (name)
		copy(name, e->name, NAME_MAX_LEN);
	if (value) {
		copy(value, e->value, PROP_VALUE_MAX);
		while (value[len])
			len++;
	}
	return len;
}

void __system_property_read_callback(const void *pi,
				     void (*callback)(void *, const char *,
						      const char *, unsigned int),
				     void *cookie)
{
	const struct entry *e = (const struct entry *)pi;

	if (e && callback)
		callback(cookie, e->name, e->value, 0);
}

int __system_property_foreach(void (*callback)(const void *, void *),
			      void *cookie)
{
	int i;

	load();
	for (i = 0; i < g_count; i++)
		callback((const void *)&g_entries[i], cookie);
	return 0;
}

/*
 * ---- liblog, redirected to stderr ----------------------------------------
 *
 * mnld reports what it is doing through Android's liblog (__android_log_print
 * and friends). The tree carries the real liblog.so, but there is no logd and no
 * /dev/socket/logd to send to, so every line disappeared - including the ones
 * that say why it gave up after the first exchange. These definitions live in
 * the preloaded shim, so they win over liblog's when the linker resolves the
 * daemon's symbols, and they print to file descriptor 2, which start.sh sends to
 * /var/log/gps-engine/mnld-<stamp>.log.
 *
 * The formatter is deliberately small: %s %c %d %i %u %x %X %p and the length
 * modifiers hh/h/l/ll/z, width and precision parsed and ignored. That is what
 * MTK's logging macros use; anything unknown is printed literally rather than
 * guessed at, and nothing here can crash the daemon.
 */
#define LOG_BUF 1024

static void out(const char *s, unsigned long n)
{
	while (n > 0) {
		long w = write(2, s, n);

		if (w <= 0)
			return;
		s += w;
		n -= (unsigned long)w;
	}
}

static unsigned long slen(const char *s)
{
	unsigned long n = 0;

	while (s && s[n])
		n++;
	return n;
}

static void append(char *dst, unsigned long *pos, const char *src,
		   unsigned long max)
{
	while (src && *src && *pos < max - 1)
		dst[(*pos)++] = *src++;
}

static void append_num(char *dst, unsigned long *pos, unsigned long long v,
		       int base, int upper, unsigned long max)
{
	char tmp[32];
	int i = 0;
	const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";

	if (v == 0)
		tmp[i++] = '0';
	while (v > 0) {
		tmp[i++] = digits[v % (unsigned long long)base];
		v /= (unsigned long long)base;
	}
	while (i > 0 && *pos < max - 1)
		dst[(*pos)++] = tmp[--i];
}

/* Format into dst; returns the length, always NUL-terminated. */
static int format(char *dst, unsigned long max, const char *tag, int prio,
		  const char *fmt, __builtin_va_list ap)
{
	static const char levels[] = "??VDIWEF";
	char p = (prio >= 0 && prio < 8) ? levels[prio] : '?';
	unsigned long pos = 0;

	dst[0] = '\0';
	if (tag) {
		append(dst, &pos, tag, max);
		append(dst, &pos, ": ", max);
	}
	if (pos < max - 1)
		dst[pos++] = p;
	if (pos < max - 1)
		dst[pos++] = ' ';

	while (fmt && *fmt && pos < max - 1) {
		int lng = 0, upper = 0;

		if (*fmt != '%') {
			dst[pos++] = *fmt++;
			continue;
		}
		fmt++;
		if (*fmt == '%') {
			dst[pos++] = *fmt++;
			continue;
		}
		/* flags, width, precision: parsed and ignored */
		while (*fmt == '-' || *fmt == '+' || *fmt == ' ' || *fmt == '#' ||
		       *fmt == '0')
			fmt++;
		while (*fmt >= '0' && *fmt <= '9')
			fmt++;
		if (*fmt == '.') {
			fmt++;
			while (*fmt >= '0' && *fmt <= '9')
				fmt++;
		}
		for (;;) {
			if (*fmt == 'h') {
				lng = 1;
				fmt++;
			} else if (*fmt == 'l') {
				lng = (lng == 3) ? 4 : 3;
				fmt++;
			} else if (*fmt == 'z' || *fmt == 'j' || *fmt == 't') {
				lng = 4;
				fmt++;
			} else {
				break;
			}
		}
		switch (*fmt) {
		case 's': {
			const char *s = __builtin_va_arg(ap, const char *);

			append(dst, &pos, s ? s : "(null)", max);
			break;
		}
		case 'c':
			dst[pos++] = (char)__builtin_va_arg(ap, int);
			break;
		case 'd':
		case 'i': {
			long long v = (lng >= 3) ? __builtin_va_arg(ap, long long)
						 : __builtin_va_arg(ap, int);

			if (v < 0) {
				if (pos < max - 1)
					dst[pos++] = '-';
				v = -v;
			}
			append_num(dst, &pos, (unsigned long long)v, 10, 0, max);
			break;
		}
		case 'u':
		case 'x':
		case 'X': {
			unsigned long long v = (lng >= 3)
				? __builtin_va_arg(ap, unsigned long long)
				: __builtin_va_arg(ap, unsigned int);

			upper = (*fmt == 'X');
			append_num(dst, &pos, v, (*fmt == 'u') ? 10 : 16, upper,
				   max);
			break;
		}
		case 'p': {
			unsigned long long v = (unsigned long long)
				(unsigned long)__builtin_va_arg(ap, void *);

			append(dst, &pos, "0x", max);
			append_num(dst, &pos, v, 16, 0, max);
			break;
		}
		case '\0':
			dst[pos++] = '%';
			break;
		default:
			dst[pos++] = '%';
			if (pos < max - 1)
				dst[pos++] = *fmt;
			break;
		}
		if (*fmt)
			fmt++;
	}
	dst[pos] = '\0';
	return (int)pos;
}

int __android_log_print(int prio, const char *tag, const char *fmt, ...)
{
	char buf[LOG_BUF];
	__builtin_va_list ap;
	int n;

	__builtin_va_start(ap, fmt);
	n = format(buf, sizeof(buf), tag, prio, fmt, ap);
	__builtin_va_end(ap);
	out(buf, (unsigned long)n);
	out("\n", 1);
	return n;
}

int __android_log_vprint(int prio, const char *tag, const char *fmt,
			 __builtin_va_list ap)
{
	char buf[LOG_BUF];
	int n = format(buf, sizeof(buf), tag, prio, fmt, ap);

	out(buf, (unsigned long)n);
	out("\n", 1);
	return n;
}

int __android_log_write(int prio, const char *tag, const char *text)
{
	char buf[LOG_BUF];
	char p = (prio >= 0 && prio < 8) ? "??VDIWEF"[prio] : '?';
	unsigned long pos = 0;

	append(buf, &pos, tag, sizeof(buf));
	append(buf, &pos, ": ", sizeof(buf));
	if (pos < sizeof(buf) - 1)
		buf[pos++] = p;
	if (pos < sizeof(buf) - 1)
		buf[pos++] = ' ';
	append(buf, &pos, text ? text : "", sizeof(buf));
	buf[pos] = '\0';
	out(buf, pos);
	out("\n", 1);
	return (int)(pos + 1);
}

int __android_log_buf_write(int bufid, int prio, const char *tag,
			    const char *text)
{
	(void)bufid;
	return __android_log_write(prio, tag, text);
}

void __android_log_assert(const char *cond, const char *tag, const char *fmt,
			  ...)
{
	char buf[LOG_BUF];
	__builtin_va_list ap;
	int n;

	__builtin_va_start(ap, fmt);
	n = format(buf, sizeof(buf), tag, 7, fmt ? fmt : "assertion failed", ap);
	__builtin_va_end(ap);
	out(buf, (unsigned long)n);
	out("\n", 1);
	if (cond && *cond) {
		out("assert: ", 8);
		out(cond, slen(cond));
		out("\n", 1);
	}
	/* The daemon is going down either way; do not return into it. */
	for (;;)
		;
}
