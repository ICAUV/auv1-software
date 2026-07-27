# SITL Notes (ArduSub simulator)

## Launch (the only correct command)

In WSL2 Ubuntu:

```bash
cd ~/ardupilot/ArduSub
../Tools/autotest/sim_vehicle.py -L RATBeach --console --out udp:127.0.0.1:14551
```

- `-L RATBeach` — **required.** Puts the vehicle in the ocean
  (California coast). The default location is CMAC, a drone airfield
  near Canberra at 584 m elevation, ON LAND: the sub sits in a field,
  motors spin (real battery drain, 12 A), vehicle never moves, and
  depth reads −584 m if you use the wrong altitude field. This cost us
  an evening; it will not cost you one.
- `--out udp:127.0.0.1:14551` — adds the endpoint our scripts listen
  on. MAVProxy `output add` typed at the prompt does NOT survive a sim
  restart; the flag does (per launch).
- `--console` — the status window (mode, ARM, Alt, battery).
- First ever run: add `-w` once to factory-wipe the simulated EEPROM.

## The two prompts

- `$` — Linux shell: launch/kill the sim, run scripts, git.
- `MAV>` — MAVProxy inside the sim terminal: talk to the vehicle
  (`arm throttle`, `disarm`, `mode manual`, `rc 3 1400`, `output add`).

Kill the sim with Ctrl+C in its terminal. Everything installed
persists across reboots; only the running sim dies.

## Conventions that bit us

- Depth = **negative** altitude (aviation heritage). Our
  `MavlinkIO.get_depth()` returns positive-down metres from
  `GLOBAL_POSITION_INT.relative_alt`. Do NOT use `VFR_HUD.alt` — that
  is absolute altitude above sea level.
- `rc 3 <pwm>`: channel 3 = vertical. 1500 neutral, <1500 down,
  >1500 up. Same PWM microsecond language as the real servos/ESCs.
- The sim vehicle is a **vectored-frame ROV** (BlueROV2-like), not our
  fin vehicle. Our DO_SET_SERVO outputs don't propel it — SITL work
  drives it via `MavlinkIO.send_manual_control()` and ArduSub's own
  mixer. Dynamics and tuned gains do NOT transfer to AUV1; loop
  structure and instincts do.
- The vehicle must be **armed** to move (`arm throttle` at MAV>, or
  `MavlinkIO.arm()`). Our scripts disarm on exit.

## Typical session

```bash
# terminal 1
cd ~/ardupilot/ArduSub
../Tools/autotest/sim_vehicle.py -L RATBeach --console --out udp:127.0.0.1:14551

# terminal 2
cd ~/auv1-software
git pull
python3 scripts/depth_hold.py 2.0
```

QGC on the Windows side usually auto-connects (sim_vehicle outputs to
the Windows host IP on 14550 under WSL2).
