# Moving control onto the Pi 5 — step-by-step

Goal: plug the Xbox pad into the laptop, and the AUV moves. Laptop and
Pi joined by the Ethernet tether for now; the same setup works over a
radio later without changing the code.

## 0. Is this the right step?

Yes — but be clear about *why*, because tethered Xbox control already
works today with `scripts/teleop.py` on the laptop. What you gain by
moving the loop onto the Pi:

- **The vehicle stays safe on its own.** Today, if the laptop script
  dies or the cable drops, the Pixhawk simply keeps the last PWM it was
  told. After the move, the Pi itself neutrals everything 0.7 s after
  the link goes quiet and disarms after 5 s.
- **The tether becomes replaceable.** Over Ethernet the laptop can send
  a 20 Hz MAVLink stream; over a 433 MHz radio it can't. A tiny
  "stick positions" packet can go over either. That's the whole reason
  the loop must live on the Pi before you go tetherless.
- **Logs are recorded on the vehicle**, so a dropped link doesn't lose
  the run.

Doing it *now*, while the tether exists, is the right order: the cable
gives you a terminal on the Pi (SSH) to debug with. Later you only
swap the transport.

## 1. The picture

```
BEFORE (today)
  Xbox ─► laptop: teleop.py ──MAVLink over tether──► BlueOS ──USB──► Pixhawk
          (reads pad, mixes fins, sends DO_SET_SERVO)

AFTER
  Xbox ─► laptop: ground_station.py ──UDP "stick packet" 20 Hz──► Pi: vehicle_daemon.py ──MAVLink (inside the Pi)──► BlueOS ──USB──► Pixhawk
          (reads pad only)              ◄──── telemetry back ────   (mixes fins, DO_SET_SERVO, watchdog, log)
```

`teleop.py` has been cut in two at the point where the stick values
exist. Four new files:

| File | Runs on | Job |
|---|---|---|
| `auv1/setpoint_link.py` | both | The "wire": sends/receives the stick packet over UDP; knows how old the last packet is |
| `scripts/ground_station.py` | laptop | Reads the Xbox pad (same pygame code as teleop.py), sends packets, prints telemetry |
| `scripts/vehicle_daemon.py` | Pi | Receives packets, runs `fin_mixer.mix`, sends `DO_SET_SERVO`, failsafes, CSV log |
| `tests/test_setpoint_link.py` | either | 4 loopback tests for the wire (`py -m pytest -q`) |

Nothing in `auv1/` that already exists is changed. `mavlink_io.py` is
still the only file that imports pymavlink.

Two words you'll meet:

- **daemon** — a program that sits in the background waiting for work,
  rather than one you interact with. The Pi script is one.
- **UDP** — the "postcard" way of sending over a network: fire and
  forget, no guarantee of delivery, no queueing. Perfect for stick
  positions, where an old value is worse than a missing one. The
  receiver simply keeps the newest postcard. `127.0.0.1` means "this
  same computer" (the *loopback* address).

## 2. Before you start

- Pi 5 powered, BlueOS booted, Pixhawk on USB, tether plugged in.
- Laptop Ethernet adapter still on the static address `192.168.2.1`
  (the July gotcha). Check: `http://192.168.2.2` opens the BlueOS page.
- **Props off.** Fins can be live; thrusters stay software-disabled
  until the very end.
- The four new files copied into the repo on Windows and committed
  (GitHub Desktop) — see step 5 for the alternative if the repo is
  private.

## 3. Step 1 — get a terminal on the Pi

**SSH** ("secure shell") gives you a command prompt *on the Pi*, typed
from your laptop. Open PowerShell on Windows:

```powershell
ssh pi@192.168.2.2
```

First time it asks "Are you sure you want to continue connecting?" —
type `yes`. Password: `raspberry` (BlueOS default; nothing appears
while you type it). The prompt changes to something like
`pi@blueos:~ $` — everything you type now runs on the Pi.

Check you're on the Pi's real operating system, not inside BlueOS's
container:

```bash
cat /etc/os-release | head -3     # expect Raspberry Pi OS / Debian
uname -m                          # expect aarch64
python3 --version                 # expect 3.11+
```

Fallback if SSH refuses: BlueOS web page → **Terminal** → type
`red-pill` → you get the same host shell. (`exit` returns.)

`exit` closes SSH. You'll want two SSH windows later (one to run the
daemon, one to look around) — just run the `ssh` command twice.

## 4. Step 2 — give the Pi internet, once

The tether alone gives the Pi no internet, and installing needs it.
Easiest: BlueOS web page → **Wi-Fi** (top bar icon) → join your phone
hotspot or home Wi-Fi. Then on the Pi:

```bash
ping -c 3 github.com               # replies = internet OK, Ctrl+C if it hangs
sudo apt update
sudo apt install -y git python3-venv python3-pip
```

(`sudo` = "do this as administrator"; `apt` = the Pi's app store.)

## 5. Step 3 — put the code on the Pi

```bash
cd ~
git clone https://github.com/ICAUV/auv1-software.git
cd auv1-software
```

If GitHub asks for a username/password, the repo is private. Either
make it public (settings → Danger zone), or create a fine-grained
personal access token (GitHub → Settings → Developer settings) with
read access to this repo and paste the token as the password. Zero-
GitHub alternative — copy straight from Windows (run in PowerShell,
not on the Pi):

```powershell
scp -r C:\projects\auv1-software pi@192.168.2.2:~/
```

Now a **virtual environment** — a private folder of Python packages
for this project, so nothing you install can disturb BlueOS:

```bash
python3 -m venv ~/venv-auv1
source ~/venv-auv1/bin/activate          # prompt now starts with (venv-auv1)
pip install -e .                         # installs pymavlink, registers auv1 package
echo 'source ~/venv-auv1/bin/activate' >> ~/.bashrc   # auto-activate in future shells
```

`-e .` means "install *this* folder as a package, editable" — so
`git pull` later updates the code with no reinstall. No pygame on the
Pi: the pad never touches it.

Quick check: `python -c "import pymavlink, auv1; print('ok')"`.

## 6. Step 4 — give the daemon its own MAVLink door in BlueOS

BlueOS is the post office that copies the Pixhawk's MAVLink stream to
everyone who asked. Today it has one *endpoint* that streams to your
laptop (`UDP Client → 192.168.2.1:14551`). Add a second one that streams
to the Pi itself:

1. BlueOS web page → toggle **Pirate mode** (top right skull) → left
   menu **Vehicle → MAVLink Endpoints**.
2. **+** → Name `auv1-daemon`, Connection type **UDP Client**,
   IP `127.0.0.1`, Port `14551`. Save. (The "Client" side is the one
   that *sends*; BlueOS sends to the Pi's own port 14551, where the
   daemon listens with `udpin:0.0.0.0:14551` — the same connection
   string `MavlinkIO` already defaults to, so nothing changes in code.)
3. Leave the existing laptop endpoint and QGC's 14550 alone. Old-style
   `teleop.py` from the laptop keeps working as a fallback.

Prove the read path with an existing script, on the Pi:

```bash
cd ~/auv1-software
python scripts/read_attitude.py        # roll/pitch/yaw scrolling = the door works
```

## 7. Step 5 — dry run on the laptop alone (5 minutes, worth it)

Before involving the vehicle, run *both halves on Windows* against the
simulator so you see the mechanics without any hardware risk. WSL
terminal: start SITL as in `docs/sitl-notes.md` (it already outputs to
Windows on 14552). Then two PowerShell windows in `C:\projects\auv1-software`:

```powershell
# window 1 — pretend to be the Pi
py scripts/vehicle_daemon.py --connect udpin:0.0.0.0:14552

# window 2 — the real ground station, pointed at this same PC
py scripts/ground_station.py --vehicle 127.0.0.1
```

Windows Firewall will pop up for Python the first time — tick **both**
private and public networks. You should see "Link up from 127.0.0.1"
in window 1 and a live status line in window 2. Press **Start**: window
1 prints "Arming ... OK", QGC shows Armed. Close window 2 with **Back**:
window 1 prints "Disarmed." Close window 2 with the ✕ instead: within a
second window 1 prints "LINK STALE" and after 5 s "Link dead —
disarming." That is the failsafe you're buying.

(The SITL vehicle won't *move* — it's a thruster ROV that ignores our
fin outputs — you're only checking the plumbing.)

## 8. Step 6 — one ArduSub parameter

ArduSub has its own "pilot vanished" failsafe: `FS_PILOT_INPUT = 2`
disarms 3 s after the last `MANUAL_CONTROL` message. Our daemon never
sends that message (it uses `DO_SET_SERVO`), so ArduSub would disarm
you 3 s after every arm. In QGC → Vehicle Setup → Parameters set
**`FS_PILOT_INPUT = 0`** (Disabled). Leave `FS_GCS_ENABLE` as is: the
daemon sends a 1 Hz GCS heartbeat itself, and the daemon's own watchdog
is now the real failsafe.

## 9. Step 7 — the real thing (props off)

Pi (SSH window):

```bash
cd ~/auv1-software
python scripts/vehicle_daemon.py
```

Laptop (PowerShell):

```powershell
cd C:\projects\auv1-software
py scripts/ground_station.py          # defaults to 192.168.2.2
```

Move the right stick: fins deflect, exactly as with `teleop.py`. Status
line shows attitude and link age (~0.05 s over Ethernet). Hold **B**:
fins centre. Let go: they follow the stick again.

Thruster test, props still off: stop the daemon (Ctrl+C — it neutrals
and disarms on the way out) and restart with `--thrusters`. Press
**Start** to arm, left stick forward gently. If the T500 responds only
when armed, arming is required; if it responds either way, keep the
arm habit anyway — it's what makes the 5 s failsafe meaningful and what
QGC displays. `THRUSTER_RANGE` is ±100 µs for now, same as teleop.py.

Logs land in `~/auv1-software/logs/` on the Pi (gitignored). Pull one
back to Windows with `scp pi@192.168.2.2:~/auv1-software/logs/vehicle_daemon_*.csv C:\projects\auv1-software\logs\` and plot with `plot_log.py`.

## 10. Step 8 (later) — start the daemon on boot

Needed once the tether goes: no SSH, so the Pi must start the daemon
itself. **systemd** is the Pi's "run this at boot and restart it if it
crashes" manager. On the Pi:

```bash
sudo tee /etc/systemd/system/auv1-daemon.service > /dev/null <<'EOF'
[Unit]
Description=AUV1 vehicle daemon (setpoints -> DO_SET_SERVO)
After=network-online.target
Wants=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/auv1-software
ExecStart=/home/pi/venv-auv1/bin/python scripts/vehicle_daemon.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now auv1-daemon      # start now + on every boot
systemctl status auv1-daemon                 # green "active (running)"
journalctl -u auv1-daemon -f                 # live output; Ctrl+C to leave
```

`sudo systemctl stop auv1-daemon` when you want to run it by hand
instead (two copies fighting over port 47001 is the classic mistake).
Add `--thrusters` to `ExecStart` only when the vehicle is ready for
water. Don't enable this until step 7 is boring.

## 11. When it doesn't work

| Symptom | Check |
|---|---|
| Daemon: "No heartbeat" | BlueOS endpoint exists, type UDP **Client**, `127.0.0.1:14551`; Pixhawk shows in BlueOS Autopilot page |
| Ground station: "NO TELEMETRY" | Daemon actually running? `ping 192.168.2.2` from the laptop? Windows Firewall allowed Python? Same `--port` on both sides? |
| Link up, fins don't move | B held? `SERVO4..7_FUNCTION` still 0 (Disabled)? Arm required by your safety switch setting? Run `test_servo.py` on the Pi to isolate |
| Sticks mapped wrong | `py scripts/teleop.py --debug` on the laptop, fix the `AXIS_*`/`BTN_*` numbers at the top of `ground_station.py` |
| Disarms 3 s after arming | `FS_PILOT_INPUT` not 0 (step 6) |
| "Address already in use" on the Pi | Another daemon (systemd?) already holds 47001: `sudo systemctl stop auv1-daemon` |
| `pip`/`python` complain about "externally managed" | The venv isn't active — `source ~/venv-auv1/bin/activate` |

## 12. What this sets up next

- **Assisted mode on the vehicle**: drop `VehicleFlightController`
  into the daemon (the packet grows two fields: depth/heading
  setpoints), leaving the laptop untouched.
- **Radio instead of Ethernet**: same packets, lower rate — raise
  `STALE_S` to match the radio's rate, nothing else changes.
- `docs/devlog.md` entry + a line in the README architecture block.
