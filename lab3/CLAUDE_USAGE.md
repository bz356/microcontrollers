# Claude Usage Log

Shared log of AI-assistant (Claude) sessions on this repo, so we have one
record across everyone's accounts instead of one shared chat account.

## How to add an entry

1. `git pull` first.
2. Add your entry at the **top of your own section** (newest first). Each
   person editing only their own section keeps merge conflicts rare.
3. Commit just this file (`git add CLAUDE_USAGE.md`) and push.

Entry template:

```markdown
### YYYY-MM-DD — labN — short title
- **Asked:** what you asked Claude to do
- **Result:** what it produced / whether we kept it
- **Files touched:** paths, or "none"
- **Commits:** hashes, if any
```

---

## Sandra

### 2026-10-29 — lab3 — 0–180 arm angle convention
- **Asked:** make the VGA show 0–180 deg, with arm down = 0, sideways = 90, up = 180
- **Result:** Added `armAngleFromAccel()` in `pwm-demo.c`: converts the raw atan2 angle (0–360) to the arm convention using two calibration constants, `RAW_ANGLE_AT_DOWN` and `ANGLE_DIRECTION`, and wraps to [-90, 270) so the arm's travel never crosses a seam. Gyro delta is multiplied by `ANGLE_DIRECTION` to match. Top-plot labels changed to 0/90/180 (the plot was already scaled 0–180 but labeled 180/360). Serial `a` command now accepts 0–180. Builds cleanly; constants still need calibrating on the rig, not hardware-tested.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`
- **Commits:** none

### 2026-10-22 — repo — usage log
- **Asked:** create a shared .md log for tracking Claude usage across partners
- **Result:** this file
- **Files touched:** `CLAUDE_USAGE.md`
- **Commits:** (this commit)

## Nat

### 2026-10-29 - lab3 - PI controller fixes and fixed startup target (Codex)

- **Asked:** Review the integral/PID implementation, explain the 1 ms timestep and holding offset, explain why the target started at approximately 92.5 degrees, keep the initialized 90-degree target at boot, fix the PI control defines, and document all changes.
- **Result:** Updated `pwm-demo.c` to use consistent PI parameters, output limits, startup state, and conditional integration. The controller remains PI; no derivative term was added.
- **Startup target:** Removed `requested_angle = fix2float15(complementary_angle)` from `main()`. The target now keeps its global initializer of 90 degrees at every boot instead of adopting the initial sensor reading. The complementary filter still starts at the measured accelerometer angle, and serial `a` can still change the target during operation.
- **Control defines:** Removed duplicate `CONTROL_MIN`/`CONTROL_MAX` definitions. Retained the user's selected `KP = 100.0f`, `KI = 3.0f`, `STARTUP_OFFSET = 4500`, `CONTROL_MIN = 1000.0f`, and `CONTROL_MAX = 5500.0f`. Runtime gains initialize from these defines. Set `KD = 0.0f`; the existing Kd display and serial setting remain available but have no effect.
- **PWM timing:** Made `WRAPVAL` an integer independent of `CONTROL_MAX`, avoiding a floating-point wrap macro in PWM APIs and integer-format display output. Set `WRAPVAL = 5999`, `CLKDIV = 25.0f`, and `CONTROL_DT = 0.001f`: at 150 MHz, `150000000 / (25 * (5999 + 1)) = 1000` updates per second. The gyro integration now uses `float2fix15(CONTROL_DT)` as well.
- **PI calculation:** `error = target - current`; the candidate accumulated error is `kerror + error * CONTROL_DT`; correction is `kp * error + ki * kerror`. Multiplying by the timestep accumulates degree-seconds rather than a sum that scales with the sample rate. This fixes the original implementation that accumulated error but returned `kp * error + ki * error`.
- **Anti-windup:** Replaced the always-true `control >= CONTROL_MIN || control <= CONTROL_MAX` check. The function evaluates the candidate command including the holding offset and integrates only when that command is within the limits, or the error moves a saturated command back toward the usable range. Setting Ki to zero clears the accumulated error so it cannot silently build up while integral control is disabled.
- **Holding command and clamp:** The motor command is `STARTUP_OFFSET + correction`, clamped to `CONTROL_MIN..CONTROL_MAX` before rounding to PWM counts. Adding a positional PI correction to the previous `control` would repeatedly accumulate the controller output. The holding offset supplies a baseline against gravity; an integral term can learn that baseline without an offset, but requires time to accumulate it. The value 4500 still needs verification on the rig.
- **Consistent initialization and plots:** Initialized `control`, `old_control`, `filtered_control`, and the actual startup PWM level from `STARTUP_OFFSET` instead of separate hard-coded 3000 values. Kept the motor plot on a separate `0..WRAPVAL` display scale. Updated stale P-only/PID comments to describe the implemented PI behavior.
- **Validation:** Pico Ninja build compiled `pwm-demo.c` and linked `PWM_Demo.elf` successfully. The sandboxed build stalled and was interrupted; the successful build ran outside the sandbox. Diff whitespace checking passed. No board flashing, hardware response testing, or gain tuning was performed; the selected gains and holding offset are not validated for stability on the rig.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Requested-angle command and five-second status (Codex)
- **Asked:** Add a requested-angle command and print filtered angle, requested angle, and current duty every five seconds; keep `doPid` P-only. Clarified that the target should be stored and reported without activating automatic motor control.
- **Result:** Added `a` followed by a finite angle from 0 through 360 degrees, accepting decimal values and rejecting invalid input. The target defaults to the initial filtered angle. Added an independent, nonblocking UART status thread with its own buffer and serialized command/status output; it snapshots the filtered angle, target, and last applied PWM duty before formatting. Updated `doPid` to `kp * (target - current)` with kp=0.5 and removed unused I/D gains. Manual duty control remains active. ARM GCC syntax checking passed. Host checks of the actual command/status functions passed for angle validation, independent duty/interval commands, five-second reporting while waiting for input, overlapping command/status output, applied-duty reporting, and P-only behavior. Host check artifacts are in the ignored build directory; hardware timing/output was not validated.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Revert attempted IMU dropout changes (Codex)
- **Asked:** Reported that IMU readings freeze or go wrong while VGA keeps refreshing, then requested "please revert those changes".
- **Result:** Reverted the interrupted dropout fix, restoring the original IMU driver, header, PWM-interrupt acquisition, filter timing, and startup sequence. Preserved the user's PID stub, requested-angle command prompt, and commented PID call, as well as the earlier filtered-angle display. Verified that the only remaining firmware diff is the user's pre-existing edits.
- **Files touched:** Restored `pwm-demo.c`, `mpu6050.c`, and `mpu6050.h`; updated `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Display filtered angle on VGA (Codex)
- **Asked:** "show on vga the filtered angle".
- **Result:** Added a live `Filtered angle: ... deg` readout above the existing plots in `pwm-demo.c`, using the existing complementary-filter result. The drawing core snapshots the fixed-point angle and formats one decimal place with integer arithmetic. The readout is drawn into the initial buffers and redrawn after each frame clear, with text size restored afterward. Preserved the existing filter and the user's ongoing edits. ARM GCC syntax checking and diff whitespace checks passed; the physical VGA output was not validated.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Review PWM application excluding VGA driver (Codex)
- **Asked:** "check the pwm demo is there anything else that is missing, we know the vga driver is a little broken so other than that".
- **Result:** Reviewed application startup, PWM updates, IMU integration, serial commands, and shared sample buffers without reviewing VGA-driver defects. Found the remaining 15-versus-16 fractional-bit mismatch, unchecked I2C failures with a near-1-ms worst-case timeout budget inside the PWM ISR, and unsynchronized cross-core access to the six sample buffers. Noted duplicate PWM definitions and GPIO14 initialization after PWM has already started. Confirmed IMU initialization precedes PWM startup, UART selection matches the serial helper, duty variables match the initial PWM setting, and the drawing core is launched. Motor duty remains manually commanded; no IMU-to-duty control algorithm is implemented. ARM GCC syntax checks passed for `pwm-demo.c` and `mpu6050.c`, with seven macro-redefinition warnings in the application. No firmware edits or hardware validation were performed.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.

### 2026-10-22 - lab3 - Remove untracked lab2 notes (Codex)
- **Asked:** Remove `../lab2/reconstructed_development_chat.md` and `../lab2/report_alignment_notes.md`.
- **Result:** Deleted exactly the two requested untracked files from lab2.
- **Files touched:** Deleted the two requested files; updated `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Recover unfinished Git merge (Codex)
- **Asked:** Help with repeated `git pull` and `git stash` failures reporting an unresolved `CLAUDE_USAGE.md` conflict.
- **Result:** Finished the pending merge after preserving the complete usage log and the staged CMake compile-command export change. Integrated the newer `origin/main` commit that restores an older copy of the log, retaining all entries when resolving the resulting add/add conflict. Untracked lab2 notes were left unchanged.
- **Files touched:** `CLAUDE_USAGE.md`; existing staged `CMakeLists.txt` change retained for the merge.
- **Commits:** merge recovery.

### 2026-10-22 - lab3 - Recheck build and numeric integration (Codex)
- **Asked:** "check again".
- **Result:** CMake configuration now succeeds for the cached Pico 2 target; merge markers, target linkage, UART selection, and the I2C include are corrected. ARM GCC syntax checking reports macro-redefinition warnings but no C errors. Found a fixed-point format mismatch: the IMU driver produces 16-fractional-bit values, while the copied macros in `pwm-demo.c` use 15 fractional bits, doubling plotted readings. Recommended removing that copied macro block and using `mpu6050.h` consistently. RGB PIO still hard-codes delays of 11/12 cycles; the changed pixelhold and vid_freq constants do not affect active instructions, so VGA pixel timing remains incorrect at 150 MHz. Monochrome plot colors, unchecked I2C results, duplicate PWM declarations, and the board default/cache mismatch remain. Actual compiler include paths select RP2350 hardware headers, not host GPIO headers. The full build stalled without output and was interrupted; no link success or hardware validation is claimed. Git reports an unresolved modify/delete conflict for this log. Firmware and CMake were reviewed without edits.
- **Files touched:** `CLAUDE_USAGE.md` only; generated build files updated by CMake.
- **Commits:** none.

### 2026-10-22 - lab3 - Check remaining integration issues (Codex)
- **Asked:** "check now is there any integration issues".
- **Result:** Confirmed I2C/IMU initialization, core-1 drawing startup, independent serial commands, and the CMake target spelling are now present. CMake still fails on merge markers at line 38. ARM GCC syntax checking fails because `pwm-demo.c` calls I2C APIs without including `hardware/i2c.h`; it also reports the existing duplicate CLKDIV macro. A final reread confirmed the user's ongoing CMake edit now enables UART and disables USB, matching the UART0 protothreads. RGB PIO takes 16 cycles per pixel, which at the cached Pico 2 default 150 MHz clock is incompatible with the 25 MHz sync timing; the driver also maps every nonzero plot color to white. Noted unchecked I2C read results and ISR timeout budget, plus the `pico` CMake default versus cached `pico2` board setting. Firmware and CMake were reviewed without edits; no hardware validation was performed.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.

### 2026-10-22 - lab3 - Implement independent serial commands (Codex)
- **Asked:** "impelment this use case".
- **Result:** Updated `protothread_serial` in `pwm-demo.c` to prompt for `d` (PWM duty, 0 through WRAPVAL) or `t` (positive plot interval in PWM interrupts), then read the selected value and return to the command prompt. Added input checks and feedback so invalid commands, nonnumeric values, trailing junk, and out-of-range values do not change settings. Initialized `control` and `old_control` to match the existing 3000 startup duty so the first zero-duty command takes effect. ARM GCC syntax checking passed with an existing duplicate CLKDIV warning. Full CMake configuration remains blocked by existing merge markers at line 38; the PWM_demo target-name typo also remains. Preserved other ongoing firmware edits.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-22 - lab3 - Explain serial command and value reads (Codex)
- **Asked:** "look at the erial thread whats the difference between the commad serial read".
- **Result:** Explained that every `serial_read` collects text into the same input buffer; the following `sscanf` determines whether that text is interpreted as an integer duty value, a command character, or an integer plot threshold. The current combined loop requests duty first, then a command, and requests a threshold only for `t`. Identified the missing `static char classifier` declaration and suggested command-based dispatch for independent duty and plot-threshold changes. No firmware edits were made.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.

### 2026-10-22 - lab3 - Recheck combined demo and build configuration (Codex)
- **Asked:** "can you check again".
- **Result:** Confirmed the VGA header, combined source list, PIO target, and DMA dependency were added. CMake configuration fails at unresolved merge markers on line 38; its library target also uses `PWM_demo` instead of `PWM_Demo`. The built application is `pwm-demo.c`, which still lacks I2C/IMU/VGA initialization and drawing-thread startup. UART protothreads remain incompatible with the disabled UART initialization, and initial PWM duty still differs from the control variables. Duplicate macro and slice declarations also remain. No firmware or CMake edits were made.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.

### 2026-10-22 - lab3 - Review combined PWM and IMU demo (Codex)
- **Asked:** "am i combing imu demo and pwm demo code rght".
- **Result:** Reviewed the combined code in `pwm-demo.c`. The shared PWM ISR contains both IMU sampling and duty updates, but the program still needs VGA and hardware headers, I2C/IMU/VGA initialization before PWM interrupts start, and a scheduled drawing thread. Identified CMake target wiring and DMA dependency problems, duplicate declarations, a mismatch between initial PWM duty and control variables, and UART-only protothreads with UART stdio disabled. Recommended one application entry point and one combined build target. Firmware was reviewed without edits or a build.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.

### 2026-10-22 - lab3 - CMake reviews and throttle explanation (Codex)
- **Asked:** "is our cmake correct in including vga graphics and imu driver"; "how about right now"; "what does throttle do, also for all prompts reference claude usage .md".
- **Result:** Reviewed two CMake revisions and identified merge markers, executable-to-executable linking, missing DMA and per-target dependencies, and a missing animation source. Explained that `throttle` counts PWM interrupts and saves one IMU sample to the plotting buffer each time it reaches `threshold`; IMU reads still happen on every interrupt. Consult `CLAUDE_USAGE.md` and record subsequent prompts and results here.
- **Files touched:** `CLAUDE_USAGE.md` only; firmware and CMake were reviewed without edits by Codex.
- **Commits:** none.

## Benson

### 2026-10-29 - lab3 - Stop disabling the IMU (Claude)
- **Asked:** "why is it not going to request angle with just p, it's like 20 degrees off"; "can you stop deactivating the IMU" / "make the fix to stop turning off the IMU now".
- **Result:** Explained P-only steady-state error: error = (u_hold - 3000) / kp. The default kp = 0.5 is far too small, and the I term (ki is stored but unused) removes the error. Removed the `imu_present` flag so the PWM ISR and startup always call `mpu6050_read_raw`. A failed reset only prints `MPU6050 did not answer on GPIO 8/9`. `mpu6050_reset()` no longer returns early when the wake-up write fails: it sends every setup write and then returns whether the wake-up write was acknowledged. Kept the SDA/SCL pull-ups, the boot print, and the 1 ms write timeouts. Not compiled.
- **Files touched:** `pwm-demo.c`, `mpu6050.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-28 - lab3 - No UART output; startup I2C hang fix (Claude)
- **Asked:** Whether lab1/2/3 print with only USB connected; then "something is wrong with the thread uart", clarified as "i just see nothing".
- **Result:** Only lab2 prints over USB (two lines after a 5 s delay); lab1's boot prints go out before USB enumerates, and lab3 uses UART0 only (GPIO 0/1). Diagnosis for lab3: `main` can hang in `mpu6050_reset()` before the protothreads start, because it used `i2c_write_blocking` with no timeout and SDA/SCL had no pull-ups (pins reset with pull-downs), so a missing IMU leaves SCL low. Also, with no IMU, each 1 kHz ISR would burn four 250 us I2C timeouts and starve core 0. Fixes: `mpu6050_reset()` now uses `i2c_write_timeout_us` (1 ms) and returns `bool` (false if the first write is not acknowledged); `pwm-demo.c` enables pull-ups on SDA/SCL, prints a boot line right after `stdio_init_all()`, stores `imu_present`, prints a warning if the IMU is missing, and skips `mpu6050_read_raw` in the ISR when it is absent. Not compiled (the build was declined) and not tested on hardware.
- **Files touched:** `mpu6050.c`, `mpu6050.h`, `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-28 - lab3 - Serial PID gain commands and pwm-demo.c cleanup (Claude)
- **Asked:** Add serial commands for the PID gains and desired beam angle (I and D can be stored without being used yet); remove the plot-interval serial command and fix the plot at about 5 s; delete dead code and update outdated comments; fix the `abs` macro; confirm serial goes through UART, not USB.
- **Result:** Serial menu is now `a` (requested angle, 0-360), `p`/`i`/`d` (Kp/Ki/Kd, any finite number >= 0); each prompts for a value, rejects bad input, and confirms. The old `d` duty command was removed because the P loop overwrote it every interrupt. Added `volatile float kp/ki/kd` globals near `requested_angle`; only `kp` is used by `doPid`. VGA status text gained a third line `Kp: ..  Ki: ..  Kd: ..`, and the text lines moved to y = 4/24/44. Removed the `t` command and `threshold` variable; plot speed is now `#define PLOT_INTERVAL 10` (530 samples x 10 ms = 5.3 s). Cleanup: deleted the commented-out filter constants, unused `vga_semaphore`, the stale "Future P control" comment, and a commented-out `printf`; rewrote the file header (purpose, merged pin list with UART0 on GPIO 0/1 and scope pins 14/15, Pico 2 instead of RP2040); replaced the stale small-angle-approximation comments in the ISR. Added parentheses to `min`/`max`/`abs` so expressions as arguments expand correctly. UART check: `lab3/CMakeLists.txt` has stdio USB 0 / UART 1, the protothread serial I/O and the status thread use `uart0` directly, and no active `printf` remains; the terminal must send CR on Enter and use 115200 baud on the adapter connected to GPIO 0/1. Confirmed the angle is not in the lab's convention (code range is 0-360, not -90 down / 0 flat); user chose to keep it. Not compiled; not validated on hardware.
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-28 - lab3 - VGA plots for beam angle and motor command (Claude)
- **Asked:** Check whether week one and week two checkpoints are met; then "let's graph these two" (actual beam angle and low-passed motor command), removing the raw accel/gyro plots, and add text showing desired angle, actual angle, and low-passed command.
- **Result:** Checkpoint review found week two missing a plotted angle, a displayed motor command, and a serial command for PID gains; noted the `d` duty command is overwritten every interrupt by the P loop (user confirmed manual duty is no longer needed). In `pwm-demo.c`: added `filtered_control`, a display-only 1/16 low-pass of `control` updated in the PWM ISR (the motor still gets raw `control`). Replaced the six raw accel/gyro plot buffers with `angle_plot` (`complementary_angle`), `target_plot` (`requested_angle`), and `control_plot` (`filtered_control`). Top plot now shows measured angle (white) and target (red) on 0/180/360 deg axes; bottom plot shows low-passed command (green) on 0/3000/6000 axes; both have titles. Added `plotY()` to scale and clamp values to the plot area, replacing the `OLD_RANGE`/`NEW_RANGE`/`ACCEL_SCALE` macros with `PLOT_HEIGHT` and per-plot min/max. Renamed `drawFilteredAngle` to `drawStatusText`, which shows `Actual: X deg  Target: Y deg` and `Command: N/6000 (P% duty)` in size-2 text. Plot positions, width, and scrolling (`t` command) are unchanged. Not compiled (CMake configure was declined) and not validated on hardware. At the time, week two still needed a serial command to set kp (added in the next entry).
- **Files touched:** `pwm-demo.c`, `CLAUDE_USAGE.md`.
- **Commits:** none.

### 2026-10-28 - lab3 - Code explanations and IntelliSense diagnosis (Claude)
- **Asked:** Explanations of `throttle` plot downsampling, the `serial_tx_busy` UART lock, the accelerometer low-pass, a line-by-line walkthrough of `on_pwm_wrap()`, and the `fix15` 15.16 fixed-point format; why Ctrl+Click doesn't work.
- **Result:** Explanations only. Notable findings: the PWM ISR runs at 1 kHz (150 MHz / 25 / 6000), so `zeropt001` matches the 1 ms timestep; the IMU I2C read takes roughly half the 1 ms budget; X-Z tilt is rotation about Y, so `gyro[2]` vs `gyro[1]` should be checked on the rig; `kp = 0.5` is likely far too small; `divfix` in `mpu6050.h` has an operator-precedence bug (unused). IntelliSense: user settings disable the C/C++ engine in favor of the STM32 clangd extension, which needs `lab3/build/compile_commands.json`; no build directory exists yet. Generating it by running CMake configure was declined, so nothing was changed.
- **Files touched:** `CLAUDE_USAGE.md` only.
- **Commits:** none.
