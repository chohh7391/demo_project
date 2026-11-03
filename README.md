# demo_project: a project on cho_robot_project

A small project that builds on the core
([cho_robot_project](https://github.com/chohh7391/cho_robot_project)) at the tag in `demo_project.repos`, the
way a research project does: its own controller, a bringup that wraps the core's, its own simulated world and what
it reads from it, and tasks the core's task manager runs. Franka FR3 in MuJoCo, at a bench with a box on it, sweeps
one joint with the project's controller (`demo_sine_sweep`), or reads the box's pose out of the simulation and stops
the TCP above it (`demo_box_reach`). What changes for another robot or the real robot, and how far the core goes
there: [Another robot, or the real robot](#another-robot-or-the-real-robot).

It edits nothing in the core. Each piece uses one of the core's extension points:

| Package | Holds | Core extension point |
| --- | --- | --- |
| `demo_controller` | `demo_controller/SineJointController`: in position mode, sweeps one joint in a sine about where it was switched in; parameters by `generate_parameter_library`; a test of the sweep | a controller plugin on the robot's base class, `FrankaBaseController` |
| `demo_bringup` | `bringup_mujoco.launch.py`: the core's Franka MuJoCo bringup in `worlds/bench.xml`, with `config/mujoco/plugins.yaml`, plus `sine_joint_controller` loaded inactive; its parameters; `config/robot_config_additions.yaml` and the environment hook that registers it | including a core bringup; `mujoco_world:=`, `mujoco_plugins:=` (the core's `BodyPosePublisher`); `CHO_ROBOT_CONFIG_ADDITIONS` |
| `demo_tasks` | the tasks `demo_sine_sweep` and `demo_box_reach` (with `StandoffPoseBehavior`), their example `task_params` files | the `cho_task_manager.tasks` entry point; `task_params:=`; `PoseTargetBehavior` on a simulated body's pose |

How the core expects a project to be laid out, and each extension point:
[templates/overlay_project/README.md](https://github.com/chohh7391/cho_robot_project/blob/main/templates/overlay_project/README.md)
and [docs/usage_guidelines.md](https://github.com/chohh7391/cho_robot_project/blob/main/docs/usage_guidelines.md).

## The workspaces

The core is built in a workspace of its own, at the pinned tag; this project's workspace overlays it:

```text
~/core_ws/src/cho_robot_project      vcs import --recursive src < demo_project.repos
~/demo_ws/src/demo_project           this repository
```

```bash
# The core: once, and again when the pin moves; build it as its docs/installation.md says
mkdir -p ~/core_ws/src && cd ~/core_ws
vcs import --recursive src < ~/demo_ws/src/demo_project/demo_project.repos

# This project, on top of it
cd ~/demo_ws
source ~/core_ws/install/setup.bash
PYTHONNOUSERSITE=1 MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test && colcon test-result --verbose
```

A core workspace already built serves as well, checked out at the pin. Every terminal that runs the project sources
`~/demo_ws/install/setup.bash`, which brings the core's with it and sets `CHO_ROBOT_CONFIG_ADDITIONS`.
`PYTHONNOUSERSITE=1` is needed when `~/.local` holds numpy 2.x: it breaks the pinocchio import.

## Running it

Two terminals, each with its own `ROS_DOMAIN_ID` if anything else on the machine runs ROS:

```bash
ros2 launch demo_bringup bringup_mujoco.launch.py
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka task:=demo_sine_sweep \
  control_mode:=position use_sim_time:=true \
  task_params:=$(ros2 pkg prefix --share demo_tasks)/config/sine_sweep.yaml
```

The task switches to `joint_space_position_controller` and goes to the registry's task home, switches to
`sine_joint_controller`, which sweeps joint 7 by 0.15 rad either side at 0.5 Hz after a 2 s ramp, lets it sweep for
`sweep_sec`, then switches the hold controller back in, which takes over the moving command and brings it to rest,
and goes home again. `/sine_joint_controller/controller_state` carries the sweep's desired and measured joints.

Where each setting is:
- the sweep (joint, amplitude, frequency, ramp, the rate it may reach):
  `demo_bringup/config/mujoco/sine_joint_controller.yaml`, ranges in
  `demo_controller/src/sine_joint_controller_parameters.yaml`;
- how long it sweeps, and the home moves: `task_params`, `demo_tasks/config/sine_sweep.yaml`.

### The world and what is read from it

The bringup gives the core's `mujoco_world:=` the demo's world, `demo_bringup/worlds/bench.xml`: a floor, a bench in
front of the arm and a 5 cm box on it, free to move. The core composes it with the FR3's model at launch, so the
world is a plain MJCF file here, with nothing of the robot in it. The FR3 stands at the world's origin: a pose in the
world is a pose in `fr3_link0`.

`mujoco_plugins:=` gets `demo_bringup/config/mujoco/plugins.yaml`, which loads the core's
`cho_simulation_mujoco/BodyPosePublisher` for the box: its pose, the simulation's ground truth, on
`/sim/body_pose/box` at 50 Hz of simulated time, stamped `fr3_link0`. A project needing other scene data writes a
plugin of its own the same way (the core's `cho_simulation_mujoco` README).

```bash
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka task:=demo_box_reach \
  control_mode:=position use_sim_time:=true \
  task_params:=$(ros2 pkg prefix --share demo_tasks)/config/box_reach.yaml
```

`demo_box_reach` goes home on the hold controller, latches the box's pose (`PoseTargetBehavior`), makes the pose
`standoff` above it with the TCP pointing down (`StandoffPoseBehavior`), moves there with `task_space_ik_controller`,
and goes home again. Move the box in `worlds/bench.xml` and the task follows it; a perceived pose, such as
cho_object_pose's, is latched the same way from its topic.

## Why the registry additions

`sine_joint_controller` claims the arm's position command interfaces, as the core's controllers do, so only one of
them can be active. Every exclusive switch deactivates the arm controllers the core's registry lists, and the
project adds its controller to that list (`controllers.additional_arm`) through
`demo_bringup/config/robot_config_additions.yaml`, which the environment hook puts on `CHO_ROBOT_CONFIG_ADDITIONS`.
Without it, the switch back to the hold, and `guarded_mission()`'s abort to it after a failure, would leave the
sweep running and fail on the claimed interfaces; `demo_sine_sweep` refuses to build against a registry that does
not list its controller.

## Another robot, or the real robot

The demo runs the FR3 in MuJoCo. A project on another arm, or on the real FR3, keeps the same three packages and
changes what in each names the robot and the backend. What follows is what the pinned core (`v0.1.0`) offers; only
the FR3 in MuJoCo has been run with this demo.

### What names the robot

| Piece | The demo (FR3) | Another robot |
| --- | --- | --- |
| The controller's base class | `cho_controller::franka::FrankaBaseController` (`cho_controller_franka`) | UR5e: `cho_controller::ur::URBaseController` (`cho_controller_ur`); FR5: `cho_controller::fr5::FR5BaseController` (`cho_controller_fr5`); OpenArm: `cho_controller::openarm::OpenArmBaseController`, or `cho_controller_openarm_mit::DirectControllerBase` for the MIT drive protocol (both in `cho_controller_openarm_mit`) |
| Its parameters | `robot_type`, `bringup_type`, `ee_name` of the FR3 in MuJoCo | the same keys, with the values that bringup's `config/<backend>/controllers*.yaml` gives the core's controllers |
| The bringup the wrapper includes | `cho_bringup_franka`, `bringup_mujoco_robot.launch.py` | `cho_bringup_<robot>`, `bringup_<backend>_robot.launch.py`; `--show-args` lists what it takes |
| Registry additions | `franka: additional_arm: [...]` | under the robot's registry name, `ur5e`, `fr5` or `openarm`; for OpenArm's bimanual build, under `profiles: <left, right or both>:`, with names starting `left_` or `right_` |
| Tasks | `TASK = {'franka': <builder>}`, run with `robot_type:=franka` | a builder under `ur5e`, `fr5` or `openarm`, run with that `robot_type:=` (and `arm:=left` or `right` for one arm of OpenArm's bimanual build) |

- The sweep is written for seven joints (`Eigen` 7-vectors). OpenArm has seven; UR5e and FR5 have six.
- UR5e and FR5 run in the position mode only.
- The tasks take the hold, the home and the base frame from the registry, so they move to another robot by adding its
  key. Their defaults name Franka's controllers: `task_space_ik_controller` is on UR5e and FR5 too, but OpenArm's
  task-space controller is `task_space_impedance_mit_controller`.
- `URBaseController` has no `nominal_period()`. The Franka, FR5 and OpenArm bases have it, and a controller that
  advances its own clock takes the period from it
  ([core CLAUDE.md, Control period](https://github.com/chohh7391/cho_robot_project/blob/v0.1.0/CLAUDE.md#control-period)).

### What each backend takes from a project

The project's controller goes in on every backend the way the demo's does: a spawner after the core's bringup, given
the controller's type and parameter file, so the core's controller list needs no entry. The rest differs by robot:

| | Franka | UR5e | FR5 | OpenArm |
| --- | --- | --- | --- | --- |
| MuJoCo: the project's world | `mujoco_world:=` | - | - (`mujoco_scene:=` picks one of the core's FR5 scenes) | - |
| MuJoCo: simulation plugins (`BodyPosePublisher`, or the project's own) | `mujoco_plugins:=` | - | - | - |
| Gazebo: a world file | `world:=` | `world_file:=` | `world_file:=` | no Gazebo bringup |
| Isaac Sim: the project's world | - | - | - | - |
| Parts between the flange and the hand, and their payload | `payload_file:=` (every backend); `ee_connection_xacro:=` (Gazebo, real); `xacro_file:=` (MuJoCo, Isaac) | - | - | - |

The Gazebo world goes to `gz sim` as given, so a project's file passes by its absolute path. It has not been tried
with a project's world.

A `-` is a hook the core does not have yet. A project that needs one adds it to the core as a general extension
point, then moves its pin to the tag that has it
([usage guidelines, The core and the projects](https://github.com/chohh7391/cho_robot_project/blob/v0.1.0/docs/usage_guidelines.md#the-core-and-the-projects)).

### On the real robot

**FR3.** The wrapper includes `cho_bringup_franka`'s `bringup_real_robot.launch.py` (`control_mode:=`,
`controller_name:=`; the robot's IP is in its `config/real/franka.config.yaml`, or `robot_config_file:=`). It spawns
the project's controller as in simulation, with `use_sim_time` false. The controller's parameter file changes:

- `bringup_type: real`. The base adds the torque terms the backend does not, so a simulation's value on the real
  robot adds gravity on top of libfranka's own compensation.
- `update_rate: 1000`, what the core's `config/real/controllers.yaml` gives each of its controllers.
- For a torque law, the end-effector payload the core gives its own controllers: `mass`, `center_of_mass` and
  `load_inertia`, from `config/payload.yaml` or the project's `payload_file:=`.

The session follows the core's
[Running the real FR3](https://github.com/chohh7391/cho_robot_project/blob/v0.1.0/docs/usage_guidelines.md#running-the-real-fr3):
Desk's end-effector setting matching what is mounted, one control mode per bringup, stages from a hold to the full
motion, and the FCI's state recorded. `sine_joint_controller` has never run on the robot, so it would start with a
hold and one short segment.

**UR5e, FR5.** Each has a `bringup_real_robot.launch.py`, position mode only. UR5e also takes `controllers_file:=`,
and FR5 takes its robot settings in `config_file:=`. The project's controller is spawned the same way. In the core's
README, UR5e's real path needs the vendor driver's setup and FR5's is still to be validated on the hardware.

**OpenArm.** The real bringup is at its commissioning stage and runs only the MIT controllers: it refuses a
`controller_name` outside `launch_utils.REAL_MIT_DIRECT_CONTROLLERS`
([openarm_real_bringup.md](https://github.com/chohh7391/cho_robot_project/blob/v0.1.0/docs/openarm_real_bringup.md)).
A project controller there would be an MIT producer on `DirectControllerBase` that follows the
[MIT contract](https://github.com/chohh7391/cho_robot_project/blob/v0.1.0/docs/openarm_mit_contract_v1.md). None has
been tried.

## Starting a project from this one

Copy the repository, then rename the packages, the controller's class and plugin name, the controller instance,
and the task, with the project's name in front and no `cho_` prefix: the core's packages have it. Keep the
environment hook, naming the new bringup package's additions file, and pin the core in the `.repos` file to the
tag the project's tests pass on.
