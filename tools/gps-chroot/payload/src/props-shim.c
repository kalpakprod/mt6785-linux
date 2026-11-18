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

/* bionic's libc, already loaded in the process. */
extern int open(const char *path, int flags, ...);
extern long read(int fd, void *buf, unsigned long count);
extern int close(int fd);

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
