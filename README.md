# Motion Planning and Collision Avoidance in CoppeliaSim Using OMPL

A CoppeliaSim simulation demonstrating motion planning, collision avoidance and redundancy exploitation for robotic manipulation using **OMPL** and **simIK**.

The simulation implements a pick-and-place task in an industrial-style workspace, where a redundant 7-DOF manipulator plans collision-free motions around obstacles. Although the current implementation uses a Sawyer manipulator, the motion-planning approach is not fundamentally tied to this robot.

![Simulation overview](/images/start_scene.png)

## Features

- Collision-free motion planning using OMPL
- Inverse kinematics using simIK
- Multiple OMPL planning algorithms
- Redundancy exploitation through multiple IK solutions
- Joint-space configuration selection for consistent motion
- Finite State Machine (FSM) task control
- Pick-and-place manipulation of different object types
- Multiple placement destinations
- Obstacle repositioning and replanning
- Automatic recovery and fallback behaviour
- Interactive object-spawning UI
- End-effector path visualisation

## Simulation

The simulation represents an industrial manipulation cell containing a conveyor system, storage shelves and multiple object destinations. Objects are detected at the pickup location before the robot plans and executes the required manipulation sequence.

The robot uses inverse kinematics to generate feasible configurations for the required end-effector pose. Collision-free motion between configurations is then planned using OMPL. The redundant 7-DOF manipulator provides alternative joint configurations for the same end-effector task, allowing the robot to adapt its posture when workspace constraints change.

The main control logic is implemented as a finite state machine:

```text
WAITING → PICKING → PLACING → SPAWNING
   ↑                              |
   └──────────────────────────────┘
```

### Interactive object selection

When the simulation is started, a small UI window allows the user to select which object types should be spawned.

![Object spawning UI](/images/start_menu.png)

This allows different task branches to be tested without modifying the script between runs.

## Requirements

- [CoppeliaSim](https://www.coppeliarobotics.com/)
- CoppeliaSim Educational version or an edition supporting the required functionality
- OMPL plugin/library
- simIK plugin/library

The simulation was developed and tested using CoppeliaSim's Lua scripting environment.

## Running the simulation

1. Clone or download this repository.
2. Open the `.ttt` scene file located in the `simulation/` directory using CoppeliaSim.
3. Start the simulation using the **Play** button.
4. Select the desired object types in the startup window.
5. Click **Run Simulation**.
6. The simulation will begin spawning objects and executing the pick-and-place task.

The robot automatically detects incoming objects, plans the required motion, performs the manipulation and returns to its home configuration before starting the next cycle.

### Testing obstacle avoidance

The workspace obstacles can be repositioned before or during testing to evaluate the planner's ability to generate alternative collision-free motions.

For example, moving an obstacle into the robot's original path causes OMPL to generate a different trajectory while the end-effector task remains unchanged.

## Project structure

```text
coppeliasim-motion-planning/
│
├── simulation/
│   └── coppeliasim_ompl_motion_planning.ttt
│
├── scripts/
│   └── main_control.lua
│   └── robot_configuration.lua
│
├── images/
│   └── *.png
│
├── report/
│   └── *.pdf
│
├── README.md
└── LICENSE
```

## Documentation

A detailed technical report describing the simulation, motion-planning approach, redundancy exploitation, experiments and HRI application is available here:

**[Motion Planning and Collision Avoidance in CoppeliaSim Using OMPL: An Industrial Manipulation Case Study](report/Motion_Planning_and_Collision_Avoidance_in_CoppeliaSim_Using_OMPL.pdf)**

The report was originally developed as part of an MSc Electronics and Robotics project at Kingston University London and has been included as supporting technical documentation for this repository.

## Code

The main threaded Lua script is provided separately in [`scripts/main_control.lua`](scripts/main_control.lua).

The script contains the main simulation control logic, including:

- IK configuration and validation
- OMPL task creation and planning
- Collision checking
- Path execution
- Redundancy-aware IK configuration selection
- Pick-and-place operations
- Object spawning
- FSM task control
- Recovery and fallback behaviour
- Simulation UI

## License

### Source Code

Copyright © 2026 Ioannis Antonakis.

The source code is released under the [MIT License](LICENSE).

### Technical Report

The accompanying technical report is licensed under
[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/).