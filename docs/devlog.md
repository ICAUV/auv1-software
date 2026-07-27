# AUV1 Engineering Log

Chronological record of work sessions, engineering decisions, and the
reasoning behind them. One entry per session, newest at the bottom.
Update this after every session — decisions without recorded reasons
get re-litigated every term.

Entry template:

```
## YYYY-MM-DD — short title
**Done:** what was accomplished
**Decisions:** what was decided and WHY (alternatives considered)
**Gotchas:** surprises / bugs / lessons that cost time
**Next:** what the following session should start with
```

---

## 2026-07-08 — Architecture decisions (pre-hardware)

**Done:** Vehicle configuration documented (SYS00 Vehicle Configuration
v1.0.docx in Teams). Control stack chosen.

**Decisions:**
- *Start with controls/teleop before other subsystems* — derisks the
  hardware integration everything else depends on; autonomy later
  replaces the human in an already-proven pipeline.
- *Stack: Pixhawk 6C (ArduSub) + RPi5 (BlueOS) + pymavlink topside* —
  "Option A phased hybrid". ArduSub provides arming/sensors/telemetry/
  failsafes free; custom control lives in our Python. Rejected:
  custom ArduSub firmware fork (Option B — too much for a solo summer),
  full ROS2 stack (Option C — schedule risk; deferred to autumn as the
  software sub-team's project with ~a weekend of migration cost thanks
  to the layering rule below).
- *Layering rule:* control maths (mixer, PIDs) is plain Python with no
  MAVLink imports; only `mavlink_io.py` touches the protocol. This is
  what makes the ROS2 migration cheap later.
- *No standard ArduSub frame fits* the vehicle (single stern thruster,
  lateral bow tunnel, 4 stern fins "+", buoyancy engine): ArduSub mixes
  thrusters only, no control surfaces. Custom mixing in our code.
- *Controller: wired Xbox One pad* (QGC-tested list). Nintendo
  untested, keyboard has no analog axes.

## 2026-07-09 — Bench bring-up: full telemetry chain

**Done:** BlueOS flashed to Pi 5 (dedicated SD; Raspberry Pi OS card
kept as fallback). ArduSub 4.5.7 flashed to Pixhawk 6C via BlueOS.
QGC connected end-to-end (6C —USB— Pi —Ethernet— PC). First pymavlink
script streaming attitude.

**Decisions:**
- *USB, not GPIO UART, for 6C↔Pi:* BlueOS only detects autopilots on
  USB (open issue #1615). TELEM2↔GPIO wiring left in place for a
  future mavlink-router/ROS2 stack.
- *Second MAVLink endpoint (UDP client → PC:14551) for our code* so QGC
  keeps the conventional 14550.

**Gotchas:**
- PC must be static 192.168.2.1. BlueOS DHCP had leased .131 and the
  telemetry endpoint streams to .1 regardless → QGC silently dead.
- 6C red B/E LED under ArduPilot is a known cosmetic bug — ignore.
- Windows Firewall must allow QGC on both network types.

## 2026-07-10..14 — Outputs, servos, repo

**Done:** GitHub org **ICAUV** created (org owned by personal accounts,
society email as org contact — no shared login). Repo `auv1-software`
scaffolded: `auv1/` package (mavlink_io, fin_mixer, controllers) +
`scripts/` + docs. All four 12V ROVMaker fin servos bench-driven via
DO_SET_SERVO.

**Decisions:**
- *Full custom control ("no frame mixing")*: all actuator outputs on
  channels ArduSub doesn't own, driven by DO_SET_SERVO. Frame
  (SimpleROV-3, placeholder) keeps MAIN 1–3; we use MAIN 4–7 (fins
  top/bottom/left/right), MAIN 8 (stern T200 ESC), AUX 1 = servo 9
  (bow T200 ESC).
- *Servo power:* 12V from bench PSU direct to servos in parallel
  (4–5 A limit); the Pixhawk-kit extension board is signal-only (its
  + rail is NOT 12V tolerant). One PSU-GND jumper to the board's −
  rail as common ground.

**Gotchas:**
- ArduSub **re-applies frame motor functions to MAIN 1–3 every boot**;
  setting them Disabled does not survive restart. DO_SET_SERVO on a
  mixer-owned output → COMMAND_ACK result=4 (FAILED).
- SERVO9 ships defaulted to Lights1 (181), SERVO10 to Mount1Pitch —
  BlueROV2 heritage. Lights freed for the bow ESC.
- MAV_RESULT codes documented in mavlink_io.wait_command_ack.

## 2026-07-15 — Teleop working

**Done:** `scripts/teleop.py`: Xbox pad → fin mixer → outputs, 20 Hz.
Control scheme: LY surge, LX yaw (fins + bow tunnel together), RY
pitch, RX roll, LB/RB depth setpoint, LT/RT pitch setpoint (VBS
placeholders, print-only until hardware exists). Safety: thrusters
software-disabled by default flag; B = instant neutral; all outputs
neutralled on any exit via try/finally.

**Decisions:**
- *Fin direction convention verified empirically, not theoretically* —
  SIGNS table in fin_mixer.py is the single home for "which way does
  this fin actually move" facts.
- *Thruster reversal is fixed in hardware* (swap two motor wires), fin
  reversal in software (SIGNS) — servos can't be reversed by wiring.

## 2026-07-26 — SITL + first closed loop (US trip, laptop only)

**Done:** ArduSub SITL running in WSL2. `depth_hold.py` — first
autonomous behaviour: dives to target and holds. Tuned in sim:
KP=0.5, KI=0.2, KD=0.7, integral_limit=1.0, MAX_EFFORT=0.6.
mavlink_io gained set_mode/arm/disarm/send_manual_control.

**Decisions:**
- *SITL drives via MANUAL_CONTROL, not DO_SET_SERVO* — the simulated
  vehicle is a standard vectored ROV; our custom outputs don't move
  it. Controllers output normalized efforts, so only the actuation
  stage differs between sim and real vehicle.
- *Depth source = GLOBAL_POSITION_INT.relative_alt* (mm, relative to
  home), not VFR_HUD.alt (absolute ASL). Consistent across sim/real.
- Sim gains are NOT vehicle gains — pool retune scheduled (roadmap
  wk 10). What transfers: loop structure and tuning instincts
  (steady-state error → more I; slow oscillation → less I more D;
  fast oscillation → less P).

**Gotchas:**
- Default SITL home is CMAC airfield, Canberra — 584 m up, **on
  land**. Sub in a paddock: motors draw 12 A, vehicle never moves.
  Launch with `-L RATBeach`. See docs/sitl-notes.md.
- MAVProxy `output add` doesn't persist across sim restarts — bake
  `--out udp:127.0.0.1:14551` into the launch command.
- Prompt discipline: `$` = Linux shell, `MAV>` = MAVProxy. Commands
  typed at the wrong one fail confusingly.

**Next:** heading_hold.py in SITL; VBS mechanism decision + ordering;
power budget. Back in HK ~Jul 27: waterproofing (roadmap wk 7),
thruster bench pulses, fin SIGNS verification session.
