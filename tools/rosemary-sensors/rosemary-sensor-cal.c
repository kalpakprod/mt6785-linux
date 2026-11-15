// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rosemary-sensor-cal — replay the factory sensor calibration into the SCP
 * sensor hub of the Poco M5s (rosemary, MT6785).
 *
 * The sensors of this phone are driven by the SCP's firmware, and without the
 * factory values the light sensor reads 0 lux and the magnetometer has no
 * bias: MIUI replays them from the nvcfg partition through the same kernel
 * interface this program uses.  The sequence and the file names are the ones
 * the MICRO sensor HAL uses (captured from its SCP log by the Redmi K30 Ultra
 * port, whose firmware is the same MediaTek sensor hub):
 *
 *   acc    acc_cali.json
 *   gyro   gyro_cali.json
 *   mag    mag_bias.json + mag_cali.json
 *   light  [0, 1000, 0...] (scale) and then als_data.json
 *   lux_b  lux_b_cali.json          (the second, front light sensor)
 *   prox   ps_cali.json             (thresholds)
 *
 * Each value goes to /dev/hf_manager as a struct hf_manager_cmd with
 * action = HF_MANAGER_SENSOR_CONFIG_CALI (the layout is the kernel's, see
 * drivers/iio/common/mtk_scp_sensors/hf_sensor_io.h).
 *
 * The calibration files are JSON objects holding one array of integers; a
 * nested array is flattened in reading order.  Source directory: the copy the
 * port ships at /lib/firmware/rosemary/nvcfg/sensor, or, when that is absent,
 * the phone's own nvcfg partition, mounted read-only.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define HF_DEV			"/dev/hf_manager"
#define HF_STATUS		"/proc/hf_manager"
#define NVCFG_PART		"/dev/disk/by-partlabel/nvcfg"
#define FW_DIR			"/lib/firmware/rosemary/nvcfg/sensor"
#define HUB_WAIT_S		60

/* the kernel's enum: action, and the sensor types of this hub */
#define HF_MANAGER_SENSOR_CONFIG_CALI	4
#define SENSOR_ACCEL			1
#define SENSOR_MAGNETIC			2
#define SENSOR_GYRO			4
#define SENSOR_LIGHT			5
#define SENSOR_PROXIMITY		8
#define SENSOR_LUX_B			81

#define CALI_VALUES			12

/* struct hf_manager_cmd of hf_sensor_io.h, packed */
struct hf_manager_cmd {
	uint8_t sensor_type;
	uint8_t action;
	int64_t delay;
	int64_t latency;
	int32_t data[CALI_VALUES];
} __attribute__((packed));

static const char *progname = "rosemary-sensor-cal";

static void log_err(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "%s: ", progname);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

/*
 * The calibration files hold one JSON object whose single member value is an
 * array of numbers (some of them nested arrays).  This reads that value and
 * flattens it; strings, booleans and nulls are skipped.
 */
static int parse_numbers(const char *text, int *out, int max)
{
	const char *p = text;
	int n = 0, arr_depth = 0, members = 0;
	bool in_value = false;

	while (*p) {
		if (*p == '"') {			/* skip a string */
			p++;
			while (*p && *p != '"') {
				if (*p == '\\' && p[1])
					p++;
				p++;
			}
			if (*p)
				p++;
			continue;
		}
		if (*p == ':') {			/* the value of a member */
			members++;
			in_value = (members == 1);
			p++;
			continue;
		}
		if (*p == '{' || *p == '}') {		/* one level of object */
			in_value = false;
			p++;
			continue;
		}
		if (*p == '[') {
			arr_depth++;
			p++;
			continue;
		}
		if (*p == ']') {
			arr_depth--;
			p++;
			continue;
		}
		if (*p == '-' || (*p >= '0' && *p <= '9')) {
			if (in_value && n < max) {
				out[n++] = (int)strtol(p, (char **)&p, 10);
			} else {
				while (*p && *p != ',' && *p != ']' &&
				       *p != '}')
					p++;
			}
			continue;
		}
		if (*p == ',') {
			/* a comma outside an array ends the value */
			if (arr_depth == 0)
				in_value = false;
			p++;
			continue;
		}
		p++;
	}

	return n;
}

static int read_calibration(const char *dir, const char *name, int *out,
			    int max)
{
	char path[512];
	char *text;
	ssize_t len;
	int fd, n;

	snprintf(path, sizeof(path), "%s/%s", dir, name);
	fd = open(path, O_RDONLY);
	if (fd < 0) {
		log_err("%s: %s", path, strerror(errno));
		return -1;
	}

	/* the files are small; read them whole */
	text = malloc(64 * 1024);
	if (!text) {
		close(fd);
		return -1;
	}
	len = read(fd, text, 64 * 1024 - 1);
	close(fd);
	if (len <= 0) {
		log_err("%s: %s", path, len ? strerror(errno) : "empty");
		free(text);
		return -1;
	}
	text[len] = '\0';

	n = parse_numbers(text, out, max);
	free(text);
	if (n <= 0) {
		log_err("%s: no numbers in the file", path);
		return -1;
	}

	return n;
}

static int send_cmd(int fd, uint8_t sensor, const int *values, int count)
{
	struct hf_manager_cmd cmd = { };

	cmd.sensor_type = sensor;
	cmd.action = HF_MANAGER_SENSOR_CONFIG_CALI;
	if (count > CALI_VALUES)
		count = CALI_VALUES;
	memcpy(cmd.data, values, count * sizeof(int));

	if (write(fd, &cmd, sizeof(cmd)) != (ssize_t)sizeof(cmd)) {
		log_err("sensor %u: %s", sensor, strerror(errno));
		return -1;
	}
	printf("sensor %u: %d calibration values sent\n", sensor, count);

	return 0;
}

/* hold on until the hub driver has registered with the SCP */
static int wait_for_hub(void)
{
	char buf[4096];
	ssize_t len;
	int fd, i;

	for (i = 0; i < HUB_WAIT_S; i++) {
		fd = open(HF_STATUS, O_RDONLY);
		if (fd >= 0) {
			len = read(fd, buf, sizeof(buf) - 1);
			close(fd);
			if (len > 0) {
				buf[len] = '\0';
				if (strstr(buf, "mtk_nanohub") &&
				    access(HF_DEV, F_OK) == 0)
					return 0;
			}
		}
		sleep(1);
	}

	log_err("the sensor hub did not come up in %d s", HUB_WAIT_S);
	return -1;
}

static int mount_partition(char *dir, size_t len)
{
	snprintf(dir, len, "/run/rosemary-sensor-cal.XXXXXX");
	if (!mkdtemp(dir)) {
		log_err("mkdtemp: %s", strerror(errno));
		return -1;
	}
	if (mount(NVCFG_PART, dir, "ext4", MS_RDONLY, NULL)) {
		log_err("mount %s: %s", NVCFG_PART, strerror(errno));
		rmdir(dir);
		return -1;
	}

	return 0;
}

int main(int argc, char **argv)
{
	const char *dir = NULL;
	char mountpoint[64], sensor_dir[128];
	bool mounted = false;
	int values[CALI_VALUES];
	struct stat st;
	int fd, n, ret = 0;

	if (argc > 2 && strcmp(argv[1], "--dir") == 0) {
		dir = argv[2];
	} else if (argc > 1 && strcmp(argv[1], "--help") == 0) {
		printf("usage: %s [--dir <nvcfg sensor directory>]\n", progname);
		return 0;
	}

	if (!dir) {
		if (stat(FW_DIR, &st) == 0 && S_ISDIR(st.st_mode)) {
			dir = FW_DIR;
		} else if (mount_partition(mountpoint, sizeof(mountpoint)) == 0) {
			snprintf(sensor_dir, sizeof(sensor_dir), "%s/sensor",
				 mountpoint);
			dir = sensor_dir;
			mounted = true;
		} else {
			return 1;
		}
	}
	printf("reading the calibration from %s\n", dir);

	if (wait_for_hub())
		return 1;
	fd = open(HF_DEV, O_RDWR);
	if (fd < 0) {
		log_err("%s: %s", HF_DEV, strerror(errno));
		return 1;
	}

	/* accelerometer */
	n = read_calibration(dir, "acc_cali.json", values, CALI_VALUES - 1);
	if (n > 0) {
		values[n] = 0;
		ret |= send_cmd(fd, SENSOR_ACCEL, values, n + 1);
	}

	/* gyroscope */
	n = read_calibration(dir, "gyro_cali.json", values, CALI_VALUES);
	if (n > 0)
		ret |= send_cmd(fd, SENSOR_GYRO, values, n);

	/* magnetometer: bias and calibration matrix go together */
	{
		int bias[CALI_VALUES], cali[CALI_VALUES];
		int nb, nc;

		nb = read_calibration(dir, "mag_bias.json", bias, CALI_VALUES);
		nc = read_calibration(dir, "mag_cali.json", cali, CALI_VALUES);
		if (nb > 0 && nc > 0) {
			if (nb + nc > CALI_VALUES)
				nc = CALI_VALUES - nb;
			memcpy(&values[0], bias, nb * sizeof(int));
			memcpy(&values[nb], cali, nc * sizeof(int));
			ret |= send_cmd(fd, SENSOR_MAGNETIC, values, nb + nc);
		}
	}

	/* main light sensor: scale first, then the calibration table */
	{
		static const int scale[CALI_VALUES] = { 0, 1000 };

		ret |= send_cmd(fd, SENSOR_LIGHT, scale, 2);
	}
	n = read_calibration(dir, "als_data.json", values, CALI_VALUES);
	if (n > 0)
		ret |= send_cmd(fd, SENSOR_LIGHT, values, n);

	/* the second, front light sensor */
	n = read_calibration(dir, "lux_b_cali.json", values, CALI_VALUES - 2);
	if (n > 0) {
		values[n] = 0;
		values[n + 1] = 0;
		ret |= send_cmd(fd, SENSOR_LUX_B, values, n + 2);
	}

	/* proximity thresholds */
	n = read_calibration(dir, "ps_cali.json", values, CALI_VALUES);
	if (n > 0)
		ret |= send_cmd(fd, SENSOR_PROXIMITY, values, n);

	close(fd);
	if (mounted) {
		umount(mountpoint);
		rmdir(mountpoint);
	}

	return ret ? 1 : 0;
}
