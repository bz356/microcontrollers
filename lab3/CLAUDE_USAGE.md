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
