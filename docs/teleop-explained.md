# teleop.py — A Complete Beginner's Walkthrough

*A line-by-line explanation of the teleoperation script: what each part
does and why it was written that way.*

---

## 0. The big picture first

`teleop.py` is a loop that runs 20 times every second. Each pass through
the loop does the same four things:

1. Ask the Xbox controller: "where are your sticks and buttons right now?"
2. Translate stick positions into *desires*: "pitch up this much, roll this much..."
3. Translate desires into servo positions (using `fin_mixer`) and send them
   to the Pixhawk (using `mavlink_io`).
4. Sleep briefly, then do it all again.

That's genuinely all a teleoperation program is. Everything else in the
file is either safety (make sure things stop when they should) or
bookkeeping (make sure we're reading the right sticks).

Why 20 times per second? Fast enough that the fins feel like they follow
your thumbs instantly (humans notice lag above ~100 ms), slow enough to
not flood the network with thousands of messages.

---

## 1. `--debug`, `sys.argv`, and what `--` means

### What happens when you type a command

When you type this in a terminal:

```
python scripts/teleop.py --debug
```

you are running the program `python` and handing it two words:
`scripts/teleop.py` and `--debug`. Python starts, runs our script, and
makes those words available to it in a list called `sys.argv`
("argv" = *argument vector*, an old C name that stuck):

```python
sys.argv == ["scripts/teleop.py", "--debug"]
```

The script checks for the flag with one line:

```python
debug = "--debug" in sys.argv
```

`in` asks "is this string anywhere in that list?" — so `debug` becomes
`True` if you typed the flag, `False` if you didn't. That's the entire
mechanism. There is no magic: `--debug` is just a word the script looks
for.

### So what is the `--` for?

Pure convention — and a very old, very universal one. Command-line
programs distinguish between:

- **positional arguments** — the *things* to operate on: `python teleop.py`
  (teleop.py is the thing)
- **options/flags** — *switches* that change behaviour: `--debug`,
  `--help`, `--version`

The `--` prefix marks a word as a switch rather than a thing. You've
already used this convention without noticing: `pip install pymavlink
--break-system-packages`, `git commit --message "..."`. Single-dash
short forms (`-m`, `-v`) are the same idea abbreviated. Programs *choose*
to honour these conventions; our script implements the tiniest possible
version (a string check). Bigger programs use Python's `argparse` library,
which adds `--help` text, error checking, and values (`--speed 5`) — worth
learning eventually, overkill today.

### What's actually different when debug is on?

Inside the main loop there's this block:

```python
if debug:
    axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
    btns = [js.get_button(i) for i in range(js.get_numbuttons())]
    print(f"axes={axes} buttons={btns}", end="\r")
    time.sleep(period)
    continue
```

If `debug` is `True`, each loop pass reads **every** axis and button,
prints them, sleeps, and then hits `continue` — which means "abandon this
loop pass here and start the next one from the top." Everything *below*
that block — stick mapping, fin mixing, servo commands — is skipped
entirely. So debug mode is physically incapable of moving anything: the
command-sending code never runs.

That's the only difference. Same program, same loop — one extra early
exit inside it.

Why does it exist? Because pygame numbers the controller's axes and
buttons generically (axis 0, axis 1, button 0...) and the numbering
varies between controllers and drivers. Debug mode lets you *see* the
numbers react as you move each control, so you can verify the constants
match your hardware before any command is trusted.

---

## 2. The docstring (lines 1–28)

The triple-quoted text at the very top. Not executed — it's the manual
page for the file: what it does, how to run it, the control layout, and
the safety behaviour. Python also stores it as the module's built-in help.
Keeping the control table here means anyone (including future-you) can
learn the controls without reading the code.

---

## 3. The imports (lines 30–38)

```python
import sys
import time
from pathlib import Path
```

Standard-library toolboxes: `sys` gives access to `sys.argv` (above) and
`sys.path` (below); `time` gives `time.sleep()`; `Path` handles file
paths.

```python
import pygame
```

The games library — we use only its joystick support, which reads the
controller through the same driver layer games use.

```python
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auv1.mavlink_io import MavlinkIO
from auv1.fin_mixer import mix, deflection_to_pwm
```

The first line is the "path trick" (explained fully in an earlier chat):
Python only searches for imports in specific places, and the repo root
isn't one of them when you run a script from `scripts/`. The line adds
the repo root to the search list so the next two imports can find the
`auv1` package. Then we import exactly the three tools we need: the
MAVLink mail room class, the fin mixing function, and the
deflection→PWM converter.

---

## 4. Configuration constants (lines 40–64)

Everything a bench operator might want to change lives at the top,
named in CAPITALS (Python's convention for "constant — set once, never
reassigned while running").

```python
THRUSTERS_ENABLED = False
```

The master safety for props. While `False`, the code *always* sends
"stop" (1500) to both thruster outputs regardless of the sticks. The
sticks still work for fins. Flip to `True` only for deliberate,
props-off thruster tests. Why in code rather than a button? So it can't
be toggled accidentally mid-session — changing it requires stopping the
program, which is exactly the friction you want around a safety.

```python
FIN_OUTPUTS = {"top": 4, "bottom": 5, "left": 6, "right": 7}
STERN_THRUSTER_OUTPUT = 8   # MAIN 8
BOW_THRUSTER_OUTPUT = 9     # AUX 1
```

The wiring map: which physical Pixhawk output each actuator is plugged
into. This must agree with reality and with the comment table in
`mavlink_io.py`. A dictionary for the fins (so code can say "the top
fin's output") and plain numbers for the two thrusters.

```python
THRUSTER_STOP = 1500
THRUSTER_RANGE = 100
```

ESC language: 1500 µs means stop; the range limits how far from stop we
ever command — ±100 µs is a gentle whisper for bench tests (the ESC
accepts up to ±400). Increasing this number is a deliberate act for later.

```python
DEADZONE = 0.12
LOOP_HZ = 20
```

`DEADZONE`: sticks never rest at *exactly* zero — a controller sitting
untouched on the desk reports tiny values like 0.03. Without a deadzone,
those tiny values would constantly twitch the fins. Anything smaller
than 0.12 is treated as "hands off". `LOOP_HZ`: the 20-times-a-second
heartbeat of the whole program.

```python
DEPTH_RATE_M_S = 0.2
DEPTH_MAX_M = 10.0
PITCH_RATE_DEG_S = 10.0
PITCH_MAX_DEG = 30.0
```

Tuning for the VBS setpoints (the future buoyancy engine): how fast the
depth target ramps while you hold LB/RB (0.2 metres per second), its
allowed range (0–10 m), and the same for the pitch-hold target
(10°/second, ±30°). Placeholders until the VBS exists, but the numbers
are real decisions you can already tune.

```python
AXIS_LX, AXIS_LY = 0, 1
AXIS_RX, AXIS_RY = 2, 3
AXIS_LT, AXIS_RT = 4, 5
BTN_B, BTN_BACK = 1, 6
BTN_LB, BTN_RB = 4, 5
```

The controller map — which pygame number corresponds to which physical
control. These are the values debug mode exists to verify. Note
`AXIS_LT/RT = 4, 5` and `BTN_LB/RB = 4, 5` don't clash: axes and buttons
are separate numbering worlds.

---

## 5. The helper functions (lines 66–90)

Small, single-job functions defined before `main()` uses them.

```python
def deadzone(x: float) -> float:
    return 0.0 if abs(x) < DEADZONE else x
```

If the stick value is tiny (either direction — `abs` ignores the sign),
report zero; otherwise pass it through unchanged. This one-liner is why
resting sticks don't twitch the fins.

```python
def trigger_01(raw: float) -> float:
    """Triggers idle at -1 and reach +1 when pulled; map to 0..1."""
    return (raw + 1.0) / 2.0
```

Quirk-absorber. Sticks report −1..+1 around a centre, but triggers
report −1 when *released* and +1 when *fully pulled* — an awkward range
for "how much is it pulled?". This remaps: released → 0, fully pulled →
1\. Walk through it: raw −1 → (−1+1)/2 = 0. raw +1 → (1+1)/2 = 1.

```python
def neutral_all(io: MavlinkIO) -> None:
    for out in FIN_OUTPUTS.values():
        io.set_servo_pwm(out, deflection_to_pwm(0.0))
    io.set_servo_pwm(STERN_THRUSTER_OUTPUT, THRUSTER_STOP)
    io.set_servo_pwm(BOW_THRUSTER_OUTPUT, THRUSTER_STOP)
```

The "make everything safe" button, as a function: every fin to centre
(deflection 0.0 → 1500 µs), both thrusters to stop. It's a function
because we call it from three different places — the B button, normal
quit, and crash cleanup — and safety code you write once is safety code
with one set of bugs to find.

---

## 6. `main()` part 1 — setup (lines 92–116)

```python
debug = "--debug" in sys.argv
```

Covered in section 1.

```python
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No controller found. Plug in / pair the Xbox controller.")
    return
js = pygame.joystick.Joystick(0)
js.init()
```

Wake up pygame, wake up its joystick system, count the controllers. Zero
controllers → print a helpful message and `return` (exit `main()`,
program over) — better than crashing later with something cryptic.
Otherwise take controller number 0 (the first one) and store the handle
in `js`, which we'll interrogate every loop.

```python
io = MavlinkIO()
print("Waiting for heartbeat...")
if not io.wait_heartbeat():
    print("No heartbeat. Check BlueOS endpoint (UDP Client -> PC:14551).")
    return
```

Open the MAVLink mail room (starts listening on UDP port 14551) and wait
for the vehicle's "I'm alive" letter. No heartbeat within 10 seconds →
explain the most likely fix and exit. The pattern here and above is
worth copying forever: **check every precondition early, fail with a
message that says what to do about it.**

```python
period = 1.0 / LOOP_HZ
depth_target_m = 0.0
pitch_target_deg = 0.0
```

`period` = 1/20 = 0.05 seconds — how long each loop pass should take.
The two `*_target` variables are the VBS setpoints; they live here
(not as constants) because they *change* while the program runs, and
they must survive from one loop pass to the next.

---

## 7. `main()` part 2 — the loop (lines 118–169)

```python
try:
    while True:
        pygame.event.pump()
```

`try:` opens the safety envelope (its partner `finally:` comes at the
end — hold that thought). `while True:` loops forever until something
`break`s out. `pygame.event.pump()` is a pygame requirement: it tells
pygame "process whatever the operating system has told you since last
time" — without this call every loop, `get_axis()` would return stale
values forever.

Then the debug block (section 1), then:

```python
if js.get_button(BTN_BACK):
    print("\nBack pressed — quitting.")
    break
if js.get_button(BTN_B):
    neutral_all(io)
    time.sleep(period)
    continue
```

Buttons return 1 (held) or 0 (not). Back → `break` = leave the loop; the
program then falls through to cleanup. B → neutral everything, skip the
rest of this pass with `continue`. Note B doesn't quit: you can hold it
as a dead-man's brake, release, and keep flying. These checks sit at the
*top* of the loop deliberately — safety gets first claim on every pass,
before any stick is read.

```python
surge = deadzone(-js.get_axis(AXIS_LY))
yaw_cmd = deadzone(js.get_axis(AXIS_LX))
pitch_cmd = deadzone(-js.get_axis(AXIS_RY))
roll_cmd = deadzone(js.get_axis(AXIS_RX))
```

Read the four stick axes and clean them up. The minus signs on the Y
axes: pygame reports stick-up as −1 (screen-coordinate heritage — Y
grows downward on screens), but we want "up = positive". Negating at
the point of reading keeps every later line intuitive.

```python
changed = False
if js.get_button(BTN_RB):
    depth_target_m = min(DEPTH_MAX_M,
                         depth_target_m + DEPTH_RATE_M_S * period)
    changed = True
```

The VBS depth setpoint. The maths: `DEPTH_RATE_M_S * period` =
0.2 × 0.05 = 0.01 m added per loop pass; at 20 passes/second that's the
advertised 0.2 m/s while held. The `min(DEPTH_MAX_M, ...)` wrapper
means "but never past 10 m" — the same clamp idiom used everywhere in
this codebase. LB does the mirror image with `max(0.0, ...)` so the
target can't go above the surface. `changed` just remembers whether
anything moved, so we only print when there's news.

```python
trim = trigger_01(js.get_axis(AXIS_RT)) - trigger_01(js.get_axis(AXIS_LT))
if abs(trim) > 0.05:
    pitch_target_deg += trim * PITCH_RATE_DEG_S * period
    pitch_target_deg = max(-PITCH_MAX_DEG,
                           min(PITCH_MAX_DEG, pitch_target_deg))
    changed = True
```

Both triggers become 0..1 values; subtracting gives one number from −1
(LT fully pulled, nose-down) to +1 (RT fully, nose-up), and 0 when both
are released or equally squeezed. Being analog, a light squeeze trims
slowly, a full pull at the maximum 10°/s. The double clamp pins the
target to ±30°.

```python
if changed:
    print(f"[VBS setpoints] depth={depth_target_m:5.2f} m  "
          f"pitch={pitch_target_deg:+6.1f} deg", end="\r")
```

Show the targets whenever they move, overwriting one line (`end="\r"`).
These numbers currently drive nothing — when the buoyancy engine
arrives, this is where its controller plugs in.

```python
fins = mix(pitch_cmd=pitch_cmd, roll_cmd=roll_cmd, yaw_cmd=yaw_cmd)
for name, deflection in fins.items():
    io.set_servo_pwm(FIN_OUTPUTS[name], deflection_to_pwm(deflection))
```

The heart. Hand the three desires to the mixer, get back a dictionary
like `{"top": 0.4, "bottom": 0.4, "left": -0.1, "right": -0.1}`. Then
for each fin: convert deflection to microseconds, look up its output
number in the wiring map, send. Notice `teleop.py` contains **zero fin
maths** — it doesn't know what a rudder is. Geometry lives in the mixer,
wiring in the map, protocol in the mail room. Each file has one job.

```python
if THRUSTERS_ENABLED:
    stern = int(THRUSTER_STOP + surge * THRUSTER_RANGE)
    bow = int(THRUSTER_STOP + yaw_cmd * THRUSTER_RANGE)
else:
    stern = bow = THRUSTER_STOP
io.set_servo_pwm(STERN_THRUSTER_OUTPUT, stern)
io.set_servo_pwm(BOW_THRUSTER_OUTPUT, bow)
```

Thrusters: full stick = stop ± 100 µs. The bow tunnel gets the *same*
yaw command as the fins — both mechanisms serve the same desire, fins
effective at speed, tunnel at rest (blending them properly by speed is a
future refinement). When the safety flag is off, both are hard-set to
stop — note the commands are still *sent* every pass, which is
deliberate: a continuous stream of "stop" is more positively safe than
silence.

```python
time.sleep(period)
```

Pause 0.05 s so the loop runs at ~20 Hz instead of thousands of times a
second (which would spam the network and peg a CPU core for no benefit).

---

## 8. `main()` part 3 — the safety envelope (lines 170–179)

```python
except KeyboardInterrupt:
    print("\nCtrl+C — quitting.")
finally:
    try:
        neutral_all(io)
        print("All outputs neutralled.")
    except Exception:
        print("WARNING: could not neutral outputs — check vehicle!")
    pygame.quit()
```

This is the other half of the `try:` from section 7, and it's the most
important safety feature in the file.

- `except KeyboardInterrupt:` — Ctrl+C in Python isn't a silent kill; it
  raises an interrupt. Catching it turns "operator panicked" into a
  clean, expected exit.
- `finally:` — the guarantee. Whatever leaves the loop — Back button,
  Ctrl+C, controller unplugged mid-run, a bug crashing the code —
  Python promises the `finally` block runs on the way out. So the last
  act of this program, under all circumstances, is *try to neutral every
  output*.
- The inner `try/except` handles the nightmare case: the cleanup itself
  fails (say, network died — which may be why we're exiting). We can't
  fix that in software, so we do the next best thing: print a loud
  warning telling the human the vehicle may be live.

The design principle: **assume the program will die at the worst
moment, and make dying safe.** Underwater vehicles are unforgiving of
code that exits with thrusters spinning.

```python
if __name__ == "__main__":
    main()
```

The run-vs-import guard (explained in an earlier chat): `main()`
executes when you run the file directly, but not if another file ever
imports something from it.

---

## 9. Ideas worth stealing for your own code

1. **Constants at the top, named in CAPITALS** — every tunable in one
   place, no magic numbers buried in logic.
2. **Check preconditions early, fail with instructions** — "No heartbeat.
   Check BlueOS endpoint..." beats a stack trace.
3. **Safety gets the top of the loop and the `finally` at the bottom** —
   first claim on every pass, last word on every exit.
4. **Absorb hardware quirks at the point of reading** (stick negation,
   trigger remap, deadzone) so the rest of the code thinks in clean units.
5. **One job per file** — this script wires a controller to a mixer to a
   mail room and adds safety; it contains no geometry and no protocol.
