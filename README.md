# TD — Modular Mapping System for Expressive Robot Puppeteering

A TouchDesigner operator library for building reconfigurable mappings between
arbitrary control inputs (MIDI, gamepad, keyboard, leader arm) and the joints of
a robotic motion rig. Operators exchange a **joint-angle vector** as their common
data format, which keeps them independent of both the controller hardware and the
robot's kinematics, so mapping structures can be rewired without touching the rest
of the chain.

This is the software artifact for:
Master Thesis: OPENING A DESIGN SPACE FOR EXPRESSIVELY PUPPETEERING A ROBOTIC MOTION RIG, Chris Hotland, 2026, University of Twente 
 >>TODO: theis link
---

## Prerequisites

### Software

%to check

| Requirement | Version | Notes |
|---|---|---|
| TouchDesigner | TODO : exact build, e.g. `2023.11xxx` | `.toe`/`.tox` files **cannot** be opened by a build older than the one that saved them. Non-commercial licence is sufficient. |
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

**Joint grouping**
%to check plz!
| Operator | Purpose |
|---|---|
| `add_overlapping` | Sums two channel groups; surplus channels are concatenated. |
| `fan_out` | Distributes one channel across multiple outputs. |
| `fan_robot_angles` | Splits a joint-angle vector into per-joint channels. |
| `scale` | Scales channel values. |
| `ref_clamp` | Clamps values against a reference. |

**Kinematics**
%to check plz!
| Operator | Purpose |
|---|---|
| `FK` | Forward kinematics. |
| `IK` | Inverse kinematics — end-effector control grouping. |
| `end_wrench` | End-effector wrench/orientation handling. |
| `retarget_movement` | Maps a movement from one rig onto another. |

**Expressive and animation layers**
%to check plz!
| Operator | Purpose |
|---|---|
| `expressive_overlay` | Second-order system exposing *anticipation* and *hold* as input ports, plus natural frequency and damping. |
| `animation` | Generative animation layer (DMP-based — see `backup/full_DMP.py`). |
| `recorder` | Records joint trajectories to `.bclip`. |
| `trigger_play` | Triggers playback of recorded movements. |
| `toggle` | Boolean switching within a chain. |
| `render` | Visual feedback of the control state. |

All angles are in **degrees** and joints are **1-indexed**.

### `full_DMP.py`

A self-contained Dynamic Movement Primitive implementation (6 DOF, 50 Gaussian
basis functions) wrapped as a Script CHOP. `imitate()` fits forcing-term weights
from a recorded trajectory by least squares; `roll_out()` regenerates it from new
start and goal states. Forcing terms are scaled by learned per-DOF amplitude
(`max(y) − min(y)`) rather than `(goal − start)`, which avoids the usual DMP
instability when start and goal coincide. The feature-extraction path is currently
commented out.

---

## Operator reference

Descriptions are from Appendix C of the thesis; connector names and parameter
defaults are read from the components themselves via `generate_operator_table.py`.
All inputs and outputs are CHOPs unless noted. Angles are in **degrees**, positions
in **cm**. The recurring `Robot` parameter takes a reference to a component from
`own_comps/robots/`, which supplies the H-matrices and twists that keep the other
operators kinematics-agnostic.

Operators are self-contained: none reads or writes anything outside itself and its
own connectors, apart from the robot configuration.

### Joint grouping

| Operator | Description | Inputs | Outputs | Parameters |
|---|---|---|---|---|
| `add_overlapping` | Adds two channel groups into one. A size difference leaves the surplus channels concatenated onto the output. | `channels1` (X ch)<br>`channels2` (Y ch) | `channels_out` (max(X,Y) ch) | - |
| `fan_out` | Separates incoming channels into single-channel outputs. | `in` (N ch) | 16 single-channel outputs: `out0_xaxis`, `out1_yaxis`, `out2_zaxis`, `out3_xrot`, `out4_yrot`, `out5_zrot`, `out6_slider1`, `out7_slider2`, `out8_b1`-`out13_b6`, `out14_p1_X`, `out15_p1_Y` | - |
| `fan_robot_angles` | Creates as many input connectors as the connected robot has joints, and merges them into one output. | `channel1` ... `channel6` | `angles` (N ch) | `Robot` (COMP) |
| `ref_clamp` | Clamps incoming channels to per-channel bounds supplied as a second input. | `clamp_vals` (2N ch, ordered `min1, max1, min2, max2, ... maxN`)<br>`in` (N ch) | `out` (N ch) | - |
| `scale` | Scales all incoming channels. Driving the `scale` input overrides the parameter. | `channels` (X ch)<br>`scale` (optional) | `channels_scaled` (X ch) | `Scale` (Float, `0.0`) |

### Kinematics

| Operator | Description | Inputs | Outputs | Parameters |
|---|---|---|---|---|
| `FK` | Computes every joint's position and orientation from the joint angles. | `angles` | `FK` (all joint positions and orientations) | `Robot` (COMP) |
| `IK` | Full control of a serial chain by setting end-effector position (cm) and orientation (degrees). Setpoints are always given in the first reference frame; which joints are driven is set by the base and end-effector parameters. `brake` outputs in the opposite direction when the chain cannot reach the setpoint - add it to an integrator (Speed CHOP) input to stop the setpoint drifting into unreachable space. | `setpoint_pos`<br>`setpoint_orient` (optional, default `[0,0,0]`) | `angles`<br>`brake` | `Endeff` - End Effector Joint (Int, `0`)<br>`Baseidx` - Base Joint (Int, `0`)<br>`Robot` (COMP)<br>`Resetrest` - Reset Rest Pose (Pulse)<br>`Gain` - velocity error gain (Int, `10`)<br>`Speedlimit` - max velocity per frame, norm (Int, `10`)<br>`Nullgain` - pull toward rest pose, i.e. elbow bending (Float, `0.2`) |
| `end_wrench` | Converts servo load readings into equivalent Fx, Fy, Fz at the chosen end-effector. | `angles`<br>`loads` | `wrench` (Fx, Fy, Fz) | `Endeff` - End Effector (Int, `0`)<br>`Robot` (COMP) |
| `render` | Wireframe render of the serial chain with endpoint and setpoint coordinate frames and the end-effector wrench. To show it behind the network editor, feed it to a Null TOP with the display flag on. | `angles`<br>`setpoint_pos`<br>`setpoint_orient`<br>`wrench` | `render` (TOP) | `Robot` (COMP) |

### Expressive and animation layers

| Operator | Description | Inputs | Outputs | Parameters |
|---|---|---|---|---|
| `expressive_overlay` | Adds dynamic character to the input channels; the viewer shows the step response for the current parameters. Driving the `damping` or `natural_freq` inputs overrides the parameters. Holding `anticipation` high pulls values back opposite to the setpoint for as long as it is held. Holding `hold` high freezes movement, then snaps to the setpoints at increased velocity on release. | `channels`<br>`anticipation` (optional)<br>`hold` (optional)<br>`damping` (optional)<br>`natural_freq` (optional) | `expressive_channels` | `Damping` (Float, `0.5`)<br>`Natfreq` - Natural Frequency (Float, `2.0`) |
| `animation` | Animates channels. Native TouchDesigner COMP. | `in` | `out` (X full-range channels) | - |
| `retarget_movement` | Abstracts the motion characteristics of a recording and reapplies them to a new target - the current channel's pose at the moment of triggering. When not triggered, the current channels pass straight through. Record start and end at the same pose to avoid a jump when the sequence fires. | `current_channels`<br>`channels_recording` (full range)<br>`trigger` | `channels` (motion sequence) | `Speedmultiplier` (Float, `1.0`) |
| `trigger_play` | Plays a full-range recording sequentially when the trigger channel rises high. | `full_range_channels`<br>`trigger` | `channels`<br>`running_flag` | - |
| `toggle` | Flips its output between 0 and 1 each time the input channel rises above 0.0. | `button_channel` | `logic` | - |
| `recorder` | TODO - not documented in Appendix C. Has no in/out connectors; it reads and writes through internal references. `Newsession` (Pulse) starts a new session. | none | none | `Newsession` (Pulse) |

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

| Operator | Inputs | Outputs | Parameters |
|---|---|---|---|
| `robot_so101` | `angles`<br>`p_gain` (optional)<br>`d_gain` (optional) | `angles_out` (measured, degrees)<br>`loads_out` (measured, Ncm) | `Nrjoints` (Int, read-only)<br>`Enabletorque` (Toggle, `True`)<br>`Com` (Int, `7`)<br>`Resetport` (Pulse)<br>`Positionpgain` (Int, `32`)<br>`Positiondgain` (Int, `32`)<br>`Acc` (Int, `50`, read-only)<br>`Speed` (Int, `2400`, read-only)<br>`Baud` (Int, `1000000`, read-only) |
| `robot_so101_MACOS_compatible` | `angles` | `angles_out` | As above, but `Comport` is a **menu** with `Refreshports` (Pulse) in place of the numeric `Com`. No `loads_out`. |
| `robot_so101_osc` | `angles` | - | `Nrjoints` (Int) |
| `robot_manual_osc` | `q_in` | - | `Nrjoints` (Int) |
| `robot_Braccio_serial` | `angles` | - | `Nrjoints` (Int)<br>`Com` (Int, `8`) |

Only `robot_so101` returns `loads_out`, which `end_wrench` requires. The macOS
variant and both OSC variants are output-only, so wrench-based feedback chains
will not work with them.

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

