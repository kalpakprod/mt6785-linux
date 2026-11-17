# MT6359 fuel gauge on rosemary: sources, design, and where to start

Research for the Battery row of the pmOS table, which currently reports no charge percentage at all. The
power supply in `/sys/class/power_supply/fuel-gauge` is the generic `adc-battery` (compatible
`adc-battery`): status, `voltage_now`, `current_now`, `charge_full_design`, no `capacity`, no `charge_now`,
no `temp`. The hold service works from voltage for that reason.

Nothing here is implemented yet. This file is the sourced ground the driver is written on, and it says why
the first piece must be the coulomb counter rather than an invented percentage.

## What the vendor does, with file:line

The vendor driver is in this repository:
`scratch/native/4-modem/src/los-mt6785/drivers/misc/mediatek/pmic/mt6359/v1/mt6359_gauge.c` (3389 lines),
with the API in `.../pmic/include/mt6359/mtk_gauge.h` and the register map in
`.../include/mt-plat/mt6785/include/mach/upmu_hw.h`.

* **Coulomb counter.** It is a 32-bit value in two PMIC registers, read as two halves and combined:
  `PMIC_FG_NCAR_15_00` / `PMIC_FG_NCAR_31_16` (`upmu_hw.h:8647-8653` and `:8651-8653`), which are
  `MT6359_FGADC_NCAR_CON0` and `_CON1` at PMIC register offsets `0xd24` and `0xd26` (`upmu_hw.h:558-559`).
  The conversion constants are in the driver: `CAR_TO_REG_SHIFT 5`, `CAR_TO_REG_FACTOR 0x2E14`,
  `UNIT_FGCAR 174080` (`mt6359_gauge.c:53,55,57`).
* **Sense resistor.** `DEFAULT_R_FG (50)` (`mt6359_gauge.c:44`): 50 mΩ, and every current path scales by
  `DEFAULT_R_FG / r_fg_value` when the board uses another value (`:258-260`, `:303-305`, `:367-373`), so the
  resistance is a device-tree property, not a constant.
* **Current.** There is no hardware current register on this path that the driver uses for reporting: the
  vendor gauge derives current from the coulomb counter over time, which is why the counter is the piece to
  get right first.
* **Voltage and temperature.** These come from the PMIC ADC (the FG's own ADC for battery voltage, the
  external NTC channel for temperature). On our kernel the ADC is already reachable through `mt6359-auxadc`
  and its IIO channels, which is where the generic `adc-battery` gets `voltage_now` today.
* **Profiles.** The vendor tables (ZCV/OCV versus capacity per temperature, and the resistance-temperature
  tables) live in the vendor's `mtk_battery_table.h` for its platforms and, for this phone, in the vendor DT's
  battery node. They are per-battery, not per-SoC: begonia and rosemary differ, so a table copied from another
  phone would produce a plausible and wrong percentage. The battery id the phone reports and rosemary's own
  profile still have to be read out of the vendor DT in our dumps before any table is used.

## What upstream has (7.3.4)

* There is **no MT6359 (or MT6358/MT6357) gauge driver** in this tree: `drivers/power/supply/` has the MT6360
  and MT6370 charger drivers and nothing for the PMIC gauge. Upstream has `simple-battery` bindings and the
  helper `power_supply_ocv2cap_simple()` (`drivers/power/supply/power_supply_core.c:1256`) for turning an open
  circuit voltage into a capacity through an `ocv-capacity-table-N` per temperature, plus
  `power_supply_battery_info` for `monitored-battery` nodes. So the shape is: a small gauge driver that
  reports coulomb count, current, voltage and temperature, and leaves the OCV-to-capacity curve to the
  generic helper and a DT `simple-battery` node describing rosemary's own battery.
* The generic `adc-battery` binding already used here is not a gauge and cannot become one: it has no
  coulomb counter and no profile.

## Design

An MFD child of the MT6359 in the same family as the clock buffer added for the NFC clock
(`drivers/clk/mediatek/clk-mt6359-clkbuf.c`): `compatible = "mediatek,mt6359-gauge"`, regmap from the parent,
one `power_supply` named **`mt6359_fg`** (decided here so the hold service can be updated in one place), with:

* `POWER_SUPPLY_PROP_CHARGE_COUNTER` and `POWER_SUPPLY_PROP_CURRENT_NOW` from the coulomb counter, with the
  sense-resistor property applied;
* `POWER_SUPPLY_PROP_VOLTAGE_NOW` from the PMIC ADC (IIO channel), and `TEMP` from the battery NTC channel;
* `POWER_SUPPLY_PROP_CAPACITY` from `power_supply_ocv2cap_simple()`, anchored on the OCV measured when the
  battery is relaxed (at boot and when `current_now` stays small), then advanced by the coulomb count while
  under load - the standard coulomb-counting-with-OCV-anchoring scheme;
* the battery profile in DT as a `simple-battery` (`monitored-battery = <&battery>`), carrying rosemary's own
  `ocv-capacity-table-0..2` and `resistance-temp-table`, which only the vendor DT in our dumps can supply.

## The first piece, and what a trial can check

In this order, because each one is checkable on its own and the next depends on it:

1. **Coulomb counter and current.** Register the supply, publish `charge_counter` and `current_now`. The trial
   compares `current_now` against the charger's own current (`mt6360-charger` reports it) while charging and
   while idle: they must agree within a few percent, which proves both the register pair and the sense
   resistor value. Nothing about capacity is claimed at this step.
2. **Voltage and temperature.** `voltage_now` must match what `adc-battery` reports today (same ADC), and
   `temp` must be plausible at room temperature and move when the phone warms - the NTC channel and its bias
   are the things to verify.
3. **OCV-anchored capacity.** Only with rosemary's own table: the trial charges to full, lets the phone rest,
   and `capacity` must read near 100 %; then a known load for a known time must move it by roughly the
   coulomb count.

Until step 1 stands, the existing voltage-based hold service stays as it is; when the driver lands, the
service switches from `voltage_now` on `fuel-gauge` to `capacity` on `mt6359_fg`, and the 45-50 % hold becomes
a percentage instead of a voltage window.
