# rosemary-sensors — factory calibration replay for the Poco M5s

The Poco M5s keeps its accelerometer, gyroscope, magnetometer, both light
sensors and the proximity sensor behind the SCP, whose firmware drives them.
The factory calibration of those parts is not in the firmware: it lives in the
`nvcfg` partition as `sensor/*.json`, and without it the light sensor reports
0 lux (its scale stays 0) and the magnetometer has no bias.

`rosemary-sensor-cal` replays it into the kernel's `/dev/hf_manager` at boot,
with the same sequence MIUI's sensor HAL uses (read back from its SCP log by
the Redmi K30 Ultra port, whose firmware is the same MediaTek sensor hub):

| sensor | files |
|---|---|
| accelerometer (type 1) | `acc_cali.json` |
| magnetometer (type 2) | `mag_bias.json` + `mag_cali.json` |
| gyroscope (type 4) | `gyro_cali.json` |
| main light (type 5) | fixed scale `[0, 1000]`, then `als_data.json` |
| front light (type 81) | `lux_b_cali.json` |
| proximity (type 8) | `ps_cali.json` (thresholds) |

Each value goes out as a `struct hf_manager_cmd` with
`action = HF_MANAGER_SENSOR_CONFIG_CALI` (4), the layout of
`drivers/iio/common/mtk_scp_sensors/hf_sensor_io.h`.

## Where the files come from

1. `/lib/firmware/rosemary/nvcfg/sensor/` when the port ships a read-only copy
   of the partition there (the `firmware` package of the port), otherwise
2. the phone's own `nvcfg` partition, mounted read-only from
   `/dev/disk/by-partlabel/nvcfg`.

`--dir <path>` overrides both.

## Build and install

Not part of the kernel build; build it on the phone or on a build host with an
aarch64 cross compiler:

    make
    make install PREFIX=/usr/local

and enable the unit:

    systemctl enable --now rosemary-sensor-cal.service

The program waits up to 60 s for the hub (`/proc/hf_manager` showing
`mtk_nanohub`) before it writes anything, so the unit can be started as soon as
the modules are loaded. It exits non-zero when the hub never appeared, and logs
each missing file — a missing `lux_b_cali.json` is not fatal for the other
sensors.
