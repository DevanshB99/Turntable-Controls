# Turntable_NEW

This folder collects the latest files for the TMC2209 + MT6701 setup in one place. Everything here is a copy; the originals are still in their old folders.

## Hardware

- **Controller:** ESP32 DevKit V1. MATLAB talks to it over USB serial at 115200 baud.
- **Motor:** NEMA17 stepper (17HE15-1504S), powered from a 12 V supply.
- **Driver:** TMC2209 v1.3 (BIGTREETECH), wired as STEP 27, DIR 26, EN 25. It runs at 1/16 microstepping (3200 steps per motor revolution) with Vref set to 1.4 V.
- **Encoder:** MT6701 14-bit magnetic encoder (16384 counts per revolution, about 0.022° per count) on I2C at address 0x06, SDA 21, SCL 22.
- **Turntable:** a belt drive with a 4:1 ratio (the motor turns 4 times for one turntable turn). On the turntable the encoder magnet sits at the centre of the disc, so it reads the turntable angle directly.
- **Arduino libraries needed:** `MT6701`, `FastAccelStepper` and `TMC2209`.

Datasheets and the ESP32 pinout are in `datasheets/`.

## Folder layout

```
Turntable_NEW/
├── tests/          small sketches to check the hardware works
├── nema17_motor/   system ID + PID tuning for the bare motor (this one was tuned)
├── turntable/      the same workflow for the belt-driven turntable (not tested yet)
├── loop_shaping/   browser page for tuning gains by loop shaping
├── results/        saved graphs
└── datasheets/
```

## Files

### tests/
| File | What it does | Originally in |
|---|-----|---|
| `MT6701_encoder_test/MT6701_encoder_test.ino` | Reads the encoder only and prints the angle. Serial commands: `RAW`, `STATS`, `RESET`, `SLOW`, `FAST`. ⚠️ It uses SDA 4 / SCL 5, unlike everything else. | `Turntable System/Turntable_Encoder_test(ArduinoxMatlab)/MT6701_Magnetic_Encoder/` |
| `TMC_MT_Motor_Test/TMC_MT_Motor_Test.ino` | Basic check that the motor turns through the TMC2209 and the encoder reads at the same time. | `Stepper Motor Controls/Arduino_Test_scripts/` |
| `NEMA17_Direction_Test/NEMA17_Direction_Test.ino` | Spins the bare motor with DIR HIGH and then DIR LOW to show which one makes the encoder angle go up. Result: HIGH = clockwise, LOW = counter-clockwise. | `Stepper Motor Controls/Arduino_Test_scripts/` |
| `Direction_Test_Turntable/Direction_Test_Turntable.ino` | The same direction check on the turntable, because the belt can flip the direction. | `Turntable System/System_ID/` |

### nema17_motor/ (bare motor)
Run the MATLAB scripts in this order:

| File | What it does | Originally in |
|---|---|---|
| `Nema17_TMC_Combined/Nema17_TMC_Combined.ino` | The one sketch to flash for every step below. It accepts velocity commands (`V…`) for system ID and runs a PID position loop every 20 ms (`HOME`, `KP`, `KI`, `KD`, `TARGET`, `START`, `STOP`, `READ`). It also has anti-windup, a derivative filter and a 3000 steps/s speed limit. | `Stepper Motor Controls/ID_Tuned/` |
| `System_ID_Nema17_TMC.m` | **Step 1.** Sends a random test signal (PRBS) to the motor, records the encoder and fits a transfer function. Saves it to `plant_model_TMC.mat`. | `Stepper Motor Controls/ID_Tuned/` |
| `RootLocus_PI_Design_TMC.m` | **Step 2.** Loads the plant and works out PI and PID gains with the root-locus method, then plots the predicted step response. Saves the gains to `pid_rootlocus_TMC.mat`. | `Stepper Motor Controls/ID_Tuned/` |
| `Hardware_PID_Tuning.m` | **Step 3.** Sends the gains to the ESP32, runs a step to `target_angle` and reports overshoot, settling time and steady-state error. Uses the controller `C` exported from pidTuner if it is in the workspace; otherwise it uses the gains last saved in the `.mat` file. | `Stepper Motor Controls/ID_Tuned/` |
| `Trajectory_Test_TMC.m` | **Step 4.** Runs a random smooth move (like a ROS2 trajectory point) and checks whether the final position is within ±0.2°. | `Stepper Motor Controls/ID_Tuned/` |
| `plant_model_TMC.mat` | The identified plant: `0.1235 / (s + 0.008139)`, fit 94.11%. | `Stepper Motor Controls/ID_Tuned/` |
| `pid_rootlocus_TMC.mat` | Saved gains: the root-locus PI/PID gains plus the last pidTuner gains used (`Kp_final`, `Ki_final`, `Kd_final`). | `Stepper Motor Controls/ID_Tuned/` |

### turntable/ (belt drive, 4:1)
Same steps as the bare motor. These files have **not been run on hardware yet**.

| File | What it does | Originally in |
|---|---|---|
| `Turntable_TMC_Combined/Turntable_TMC_Combined.ino` | The same firmware as the motor version, but it works in turntable degrees (it takes the 4:1 ratio into account) and limits speed to 1000 steps/s to start with. | `Turntable System/System_ID/` |
| `System_ID_Turntable.m` | Step 1: identifies the turntable plant and saves it to `plant_model_turntable.mat`. | `Turntable System/System_ID/` |
| `RootLocus_PI_Design_Turntable.m` | Step 2: root-locus gains for the turntable. | `Turntable System/Hardware_Tuning/` |
| `Hardware_PID_Tuning_Turntable.m` | Step 3: step test on the turntable. | `Turntable System/Hardware_Tuning/` |
| `Trajectory_Test_Turntable.m` | Step 4: random trajectory test on the turntable. | `Turntable System/Hardware_Tuning/` |

> **Note:** `Hardware_PID_Tuning.m` and `Hardware_PID_Tuning_Turntable.m` have the **old folder path written into them** (`scriptDir`, line 7), so they still read and write the `.mat` files in the old folders. Change that line to use this folder. The other scripts use their own folder automatically.

### loop_shaping/
| File | What it does | Originally in |
|---|---|---|
| `Loop_Shaping_TMC_MT6701.html` | Open it in a browser (no install needed) to tune Kp/Ki/Kd by loop shaping. It shows the Bode plot, phase margin, simulated step response and closed-loop poles as you drag the gains. It also scores the Ziegler–Nichols-style rules and the SIMC rule on the **Auto-tune** tab. Use it alongside pidTuner, then test the gains with `Hardware_PID_Tuning.m`. | Harry's `Turntable_Loop_Shaping.html` (`Harry Git/Turntable_Hybrid/Turntable/PID/`), adapted to this hardware |

**What was changed for this hardware:** the controller now runs on the ESP32 every 20 ms (not on the PC every 10 ms), and the output is in microsteps/s (not °/s). The page uses the 16384-count MT6701 and models `Nema17_TMC_Combined.ino` exactly: derivative of the error with an EMA filter (α = 0.2, so Tf = 0.08 s), conditional anti-windup, a ±100 integral clamp and the `MAX_STEP_RATE` clamp. A **System** switch picks the bare motor or the 4:1 turntable.

**Plant constants and where they come from:**

| Constant | NEMA17 | Turntable 4:1 | Source |
|---|---|---|---|
| Ka (°/s per step/s) | 0.1235 | 0.0281 (theory) | K in `sys_p = K/(s+p)` from system ID. For the turntable, type in K once `System_ID_Turntable.m` has been run |
| Td (s) | 0.005 | 0.005 | The MT6701 library refreshes the angle every 10 ms, so it is about 5 ms old on average |
| τa (s) | 0.005 | measure | No acceleration ramp in the firmware; too small to show up in the bare-motor system ID |
| τf (s) | 0 | 0 | The firmware does not filter the angle |
| Max rate (steps/s) | 3000 | 1000 | `MAX_STEP_RATE` in the `.ino` |

**Check against hardware:** with *Include hardware limits* on, the page predicts 2.5% overshoot and a 0.36 s rise for the best gains on a 130° step. The real run gave 2.7% and 436 ms.

### results/nema17_motor/
These are graphs from the last bare-motor runs. The scripts overwrite them on every run.

- **System ID:** `position_plant_validation.png`, `response_comparison.png`, `velocity_gain_check.png`, `bode.png`, `root_locus.png`, `closed_loop_continuous.png`, `closed_loop_discrete.png`
- **Root-locus design:** `rl_root_locus*.png`, `rl_pi_step_response.png`, `rl_pi_vs_pid_step_response.png`, `rl_zeta_sweep_overshoot.png`, `rl_td_ratio_sweep_overshoot.png`, `rl_bode_comparison.png`
- **Derivative filter (phase lag check):** `rl_filtered_derivative_*.png`
- **Hardware trajectory:** `trajectory_test.png`. A move of about −150° in about 1.75 s. The tracking error peaks at about 10° during the move, oscillates by a few degrees until about 3.8 s, then settles close to 0. The gains used for this run were not recorded.

## PID / PI gains tried (bare NEMA17 + TMC2209 + MT6701, July 2026)

Plant from system ID: `0.1235 / (s + 0.008139)`, fit 94.11%.

| Kp / Ki / Kd | Where they came from | Result |
|---|---|---|
| 0.0943 / 0.00139 / 0 | `C_pid` from the system ID script | Motor barely moved; error 173.6° |
| 5 / 0.2, 12 / 1.12, 12 / 15.18 | Set by hand while the old firmware was stuck at 400 steps/s | Error 36–46° (firmware problem, fixed afterwards) |
| 64.77 / 202.42 / 0 | Root locus, ζ = 0.8, ωn = 5 | Predicted 17.9% overshoot (not run on hardware) |
| 64.77 / 129.55 / 0 | Root locus, ζ = 1.0, ωn = 4 | Predicted 13.5% overshoot |
| 90.68 / 181.37 / 3.24 | Root-locus PID | 130° step: 33.1% overshoot, 0.1° error, settling 731 ms |
| 235.71 / 700.62 / 0.827 | pidTuner | 180° step: 43.3% overshoot, 0.6° error |
| 294.8 / 1096 / 0.827 | pidTuner | 130° step: 55.5% overshoot, 0.0° error |
| **69.8 / 61.435 / 0.827** | **pidTuner, after adding anti-windup** | **130° step: 2.7% overshoot, 0.4° error, rise 436 ms, settling 478 ms ✅ best recorded** |

The derivative filter was added to the firmware after the best run, so test 69.8 / 61.435 / 0.827 again before relying on it.

The 33%, 43% and 55% overshoot runs were all made **before anti-windup was added**. The loop-shaping page reproduces them (26–54%) only with anti-windup switched off, so integral windup caused them, not the gains alone. With today's firmware those gains would likely overshoot much less, so they are worth retesting.
