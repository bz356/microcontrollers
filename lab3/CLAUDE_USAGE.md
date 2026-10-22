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

### 2026-10-22 — repo — usage log
- **Asked:** create a shared .md log for tracking Claude usage across partners
- **Result:** this file
- **Files touched:** `CLAUDE_USAGE.md`
- **Commits:** (this commit)

## Nat

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

_No entries yet._
