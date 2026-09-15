# TD — Modular Mapping System for Expressive Robot Puppeteering

A TouchDesigner operator library for building reconfigurable mappings between
arbitrary control inputs (MIDI, gamepad, keyboard, leader arm) and the joints of
a robotic motion rig. Operators exchange a **joint-angle vector** as their common
data format, which keeps them independent of both the controller hardware and the
robot's kinematics, so mapping structures can be rewired without touching the rest
of the chain.

This is the software artifact for the publication: OPENING A DESIGN SPACE FOR EXPRESSIVELY PUPPETEERING A ROBOTIC MOTION RIG, C. Hotland and N. Kousi, 2026, University of Twente 
---

## Prerequisites

### Software

| Requirement | Version | Notes |
|---|---|---|
| TouchDesigner | `2025.32050` | `.toe`/`.tox` files **cannot** be opened by a build older than the one that saved them. Non-commercial licence is sufficient. |
| Python | 3.11 | Must match the interpreter your TouchDesigner build ships with. Check with `import sys; print(sys.version)` in the Textport. |
| Git | any | Recommended over ZIP download : see the folder-name warning below. |

### Hardware

The system is robot-agnostic, but the operators in `own_comps/robots/` are written
for specific targets:

- **SO-101 arm** with Waveshare ST3215 servos, the primary platform used in the
  study. Connects over USB serial via `feetech-servo-sdk`.
- **Arduino Braccio** via robot_Braccio_serial.tox.
- **Any OSC-capable target**  via `robot_so101_osc.tox` / `robot_manual_osc.tox`.

Controller inputs used during development: MIDI launchpad, gamepad, keyboard, and
a second SO-101 used as a teleoperation leader arm. None of these are required 
any TouchDesigner input source can be wired in.

You can open the project and explore the mapping chains without any robot
connected; only the `robot` operator needs hardware.

---

## Installation

### 1. Get the files

```bash
git clone https://github.com/ChrisHoltland/TD.git
cd TD
```

> **The root folder must be named exactly `TD`.** The project resolves the virtual
> environment relative to this name. If you downloaded a ZIP, it extracts as
> `TD-main` -> rename it to `TD` before continuing, or nothing will initialise.

### 2. Create the virtual environment

**Windows**

```cmd
python -m venv TD_vEnv
TD_vEnv\Scripts\activate
pip install -r requirements.txt
```

(PowerShell: `.\TD_vEnv\Scripts\activate.ps1`)

**macOS**

```bash
python3 -m venv TD_vEnv
```

Before installing, apply the macOS path fix. TouchDesigner on macOS looks for
`site-packages` under a version-named folder, which `venv` does not create:

```bash
mkdir -p TD_vEnv/lib/python3.11
mv TD_vEnv/lib/site-packages TD_vEnv/lib/python3.11/
```

Replace `python3.11` with whatever `print(sys.version)` reports in your Textport.
Then:

```bash
source TD_vEnv/bin/activate
pip install -r requirements.txt
```

You'll know the environment is active when `(TD_vEnv)` appears at the front of your
terminal prompt.

### 3. Point TouchDesigner at it

1. Open `conference.toe`.
2. **Edit → Preferences → Python**  set the Python Module Path to your `TD` folder.
3. Open the Textport (`Alt+T` / `Cmd+T`). No initialisation errors means you're set.

> `TDPyEnvManagerContext.json` contains an absolute path from the original
> development machine. Edit `installPath` to your own `TD` directory if the
> environment manager doesn't pick up the venv.

### 4. Connect a robot

1. Select the `robot` operator and press `P` to open its parameters.
2. Choose the correct COM (Windows) or `/dev/tty.*` (macOS) serial port.
3. Enable **Torques**.
4. Double-click the operator to force it to cook.

---

## Repository structure

```
TD/
├── conference.toe              # main project file
├── demo.12.toe                 # TODO — appears identical to conference.toe; remove or document
├── requirements.txt
├── TDPyEnvManagerContext.json  # venv config (contains a machine-specific path)
├── own_comps/
│   ├── blocks/                 # the operator library, see below
│   ├── robots/                 # robot output operators (SO-101, Braccio, OSC)
│   ├── backup/                 # earlier operator versions + full_DMP.py
│   └── images/
├── TD_assets/
│   └── Keyframer.*.tox         # third-party keyframer component
├── TDImportCache/              # cached FBX geometry for the background robot view (~23 MB)
└── dataset/
    └── backaway_degrees.bclip  # example recorded movement clip
```

---

## Operator reference

Angles are in **degrees**, positions in **cm**.
The recurring `Robot` parameter takes a reference to a component from
`own_comps/robots/`, which supplies the H-matrices and twists that keep the other
operators kinematics-agnostic.

Operators are self-contained: none reads or writes anything outside itself and its
own connectors, apart from the robot configuration. Connector names are visible in
TouchDesigner by hovering over an operator's inputs and outputs.

### Joint grouping

| Operator | Description | Parameters |
|---|---|---|
| `add_overlapping` | Adds two channel groups into one. A size difference leaves the surplus channels concatenated onto the output. | - |
| `fan_out` | Separates incoming channels into single-channel outputs. Currently laid out for a 16-control device: 3 axes, 3 rotations, 2 sliders, 6 buttons, 2 pad axes. | - |
| `fan_robot_angles` | Creates as many input connectors as the connected robot has joints, and merges them into a single output. | `Robot` (COMP) |
| `ref_clamp` | Clamps incoming channels to per-channel bounds supplied as a second input, ordered `min1, max1, min2, max2, ... maxN`. | - |
| `scale` | Scales all incoming channels. Driving the scale input overrides the parameter. | `Scale` (Float, `0.0`) |

### Kinematics

| Operator | Description | Parameters |
|---|---|---|
| `FK` | Computes every joint's position and orientation from the joint angles. | `Robot` (COMP) |
| `IK` | Full control of a serial chain by setting end-effector position and orientation. Setpoints are always given in the first reference frame; which joints are driven is set by the base and end-effector parameters. The brake output pushes in the opposite direction when the chain cannot reach the setpoint - add it to an integrator (Speed CHOP) input to stop the setpoint drifting into unreachable space. | `Endeff` - End Effector Joint (Int, `0`)<br>`Baseidx` - Base Joint (Int, `0`)<br>`Robot` (COMP)<br>`Resetrest` - Reset Rest Pose (Pulse)<br>`Gain` - velocity error gain (Int, `10`)<br>`Speedlimit` - max velocity per frame, norm (Int, `10`)<br>`Nullgain` - pull toward rest pose, i.e. elbow bending (Float, `0.2`) |
| `end_wrench` | Converts servo load readings into equivalent Fx, Fy, Fz at the chosen end-effector. | `Endeff` - End Effector (Int, `0`)<br>`Robot` (COMP) |
| `render` | Wireframe render of the serial chain with endpoint and setpoint coordinate frames and the end-effector wrench. To show it behind the network editor, feed it to a Null TOP with the display flag on. | `Robot` (COMP) |

### Expressive and animation layers

| Operator | Description | Parameters |
|---|---|---|
| `expressive_overlay` | Adds dynamic character to the input channels; the viewer shows the step response for the current parameters. Driving the damping or natural frequency inputs overrides the parameters. Holding anticipation high pulls values back opposite to the setpoint for as long as it is held. Holding hold high freezes movement, then snaps to the setpoints at increased velocity on release. | `Damping` (Float, `0.5`)<br>`Natfreq` - Natural Frequency (Float, `2.0`) |
| `animation` | Animates channels. Native TouchDesigner COMP. | - |
| `retarget_movement` | Abstracts the motion characteristics of a recording and reapplies them to a new target - the current channel's pose at the moment of triggering. When not triggered, the current channels pass straight through. Record start and end at the same pose to avoid a jump when the sequence fires. | `Speedmultiplier` (Float, `1.0`) |
| `trigger_play` | Plays a full-range recording sequentially when the trigger channel rises high. | - |
| `toggle` | Flips its output between 0 and 1 each time the input channel rises above 0.0. | - |
| `recorder` | TODO - not documented in Appendix C. Has no in/out connectors; it reads and writes through internal references. | `Newsession` (Pulse) |

### Robot operators (`own_comps/robots`)

The robot operator prepares and sends data to the connected robot or OSC target,
and holds the configuration information (H0-matrices and twists) the other
operators read. It is robot-specific by design: to support different hardware,
adapt its configuration and communication scripts.

Specific to the SO-101: `Com` selects the USB port, and `Resetport` closes and
reopens it. Disabling torques stops the servos actuating toward their setpoints
while still taking measurements, which is what makes hand-puppeteering during
recording possible. The P and D parameters set each servo's internal gains, and
driving the corresponding inputs overrides them. Number of joints is derived from
the configuration information; acceleration, speed and baud reflect the
communication protocol.

| Operator | Parameters |
|---|---|
| `robot_so101` | `Nrjoints` (Int, read-only)<br>`Enabletorque` (Toggle, `True`)<br>`Com` (Int, `7`)<br>`Resetport` (Pulse)<br>`Positionpgain` (Int, `32`)<br>`Positiondgain` (Int, `32`)<br>`Acc` (Int, `50`, read-only)<br>`Speed` (Int, `2400`, read-only)<br>`Baud` (Int, `1000000`, read-only) |
| `robot_so101_MACOS_compatible` | As above, but `Comport` is a menu with `Refreshports` (Pulse) in place of the numeric `Com`. |
| `robot_so101_osc` | `Nrjoints` (Int) |
| `robot_manual_osc` | `Nrjoints` (Int) |
| `robot_Braccio_serial` | `Nrjoints` (Int)<br>`Com` (Int, `8`) |

Only `robot_so101` returns measured servo loads, which `end_wrench` requires. The
macOS variant and both OSC variants are output-only, so wrench-based feedback
chains will not work with them.

### `full_DMP.py`

A self-contained Dynamic Movement Primitive implementation (6 DOF, 50 Gaussian
basis functions) wrapped as a Script CHOP. `imitate()` fits forcing-term weights
from a recorded trajectory by least squares; `roll_out()` regenerates it from new
start and goal states. Forcing terms are scaled by learned per-DOF amplitude
(`max(y) − min(y)`) rather than `(goal − start)`, which avoids the usual DMP
instability when start and goal coincide. The feature-extraction path is currently
commented out.

---

## Troubleshooting

**Textport says virtual environment `TD_vEnv` not found**

- Is the root folder named exactly `TD`? Rename it if not.
- On macOS, confirm the path `TD/TD_vEnv/lib/python3.x/site-packages` exists. If
  not, apply the folder fix in step 2 above using the version reported by
  `print(sys.version)`.

**Robot operator isn't updating**

- Double-click the `robot` operator to force a cook.

**Physical robot unresponsive**

- Torques enabled? Select `robot` → `P` → enable Torques.
- Correct serial port? Refresh the port list in the parameters and try another.
- Hardware locked or in an error state? Physically nudge the robot to clear dead
  zones and joint limits.
- Still nothing? Power-cycle: reconnect the supply and reseat the USB cable.

---

## Known limitations

- Runs at ~20 fps, capped by the robot operator's communication protocol. Adequate
  for real-time control but not for high-bandwidth teleoperation.
- `.toe` and `.tox` files are compressed binaries: they cannot be diffed in Git,
  and the saving build cannot be read without opening them in TouchDesigner.
- Large mapping structures become visually hard to manage in a node-based editor.
- Only one example movement clip is included.

