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
