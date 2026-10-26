# demo_project: a project on cho_robot_project

A small project that builds on the core
([cho_robot_project](https://github.com/chohh7391/cho_robot_project)) at the tag in `demo_project.repos`, the
way a research project does: its own controller, a bringup that wraps the core's, its own simulated world and what
it reads from it, and tasks the core's task manager runs. Franka FR3 in MuJoCo, at a bench with a box on it, sweeps
one joint with the project's controller (`demo_sine_sweep`), or reads the box's pose out of the simulation and stops
the TCP above it (`demo_box_reach`).

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

## Starting a project from this one

Copy the repository, then rename the packages, the controller's class and plugin name, the controller instance,
and the task, with the project's name in front and no `cho_` prefix: the core's packages have it. Keep the
environment hook, naming the new bringup package's additions file, and pin the core in the `.repos` file to the
tag the project's tests pass on.
