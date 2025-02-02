# Cooperative Robotics Experiments

## Overview
This repository contains MATLAB implementations for various cooperative robotics experiments. The experiments focus on task prioritization, motion planning, and control of robotic systems, including underwater vehicles and robotic manipulators.

## Authors
- **Dikshant Thakur** - s5943225@studenti.unige.it
- **Ouassim Milous** - s5938924@studenti.unige.it
- **Girum Molla Desalegn** - s6020433@studenti.unige.it

## Structure
```
Cooperative-Robotics-Experiments/
├── ex1/       # Experiment 1: Implementing a Complete Mission
├── ex2/       # Experiment 2: Bimanual Manipulation
├── ex3/       # Experiment 3: Cooperative Manipulation
├── report/    # Report and documentation
```

### Experiment 1: Implementing a Complete Mission
#### Objective
- Implement several actions to control an underwater vehicle with a manipulator.
- Reach a designated point, land, and manipulate a target object while maintaining constraints.

#### Key Actions
- **A1 - Safe Waypoint Navigation**
- **A2 - Nodule Alignment**
- **A3 - Safe Landing**
- **A4 - Fixed-Base Manipulation**

#### Key Files
- `MainRobust.m` - Main simulation script.
- `ComputeJacobians.m` - Computes Jacobians for control.
- `ComputeTaskReferences.m` - Computes task references.
- `ActionTransition.m` - Manages action transitions.
- `UpdateMissionPhase.m` - Handles phase transitions.

### Experiment 2: Bimanual Manipulation
#### Objective
- Implement a task-priority algorithm for two manipulators working together as a single robot.
- Compute transformations and coordinate movement.

#### Key Actions
- **A1 - Move-to**
- **A2 - Cooperative Move-to**
- **A3 - Halt Manipulator**

#### Key Files
- `main.m` - Main simulation script.
- `ComputeJacobians.m` - Computes Jacobians for control.
- `ComputeTaskReferences.m` - Computes task references.
- `ActionTransition.m` - Manages action transitions.
- `UpdateMissionPhase.m` - Handles phase transitions.

### Experiment 3: Cooperative Manipulation
#### Objective
- Implement cooperative manipulation for two robotic arms as separate systems.
- Maintain independent task-priority kinematics while ensuring coordinated movement.

#### Key Actions
- **A1 - Move-to (Independent motion)**
- **A2 - Cooperative Move-to (Synchronized motion)**
- **A3 - Halt Manipulator**

#### Key Files
- `main.m` - Main simulation script.
- `ComputeJacobians.m` - Computes Jacobians for control.
- `ComputeTaskReferences.m` - Computes task references.
- `ActionTransition.m` - Manages action transitions.
- `UpdateMissionPhase.m` - Handles phase transitions.

## Running the Code
1. Clone the repository:
   ```sh
   git clone https://github.com/OuassimMilous/Cooperative-Robotics-Assignment.git
   ```
2. Open MATLAB and navigate to the appropriate experiment folder.
3. Run the main script:
   - `MainRobust.m` for Experiment 1
   - `main.m` for Experiments 2 and 3

## Dependencies
- MATLAB (Tested on R2023b and R2024b)
- Robotic System Toolbox


## Acknowledgments
Special thanks to the University of Genoa for providing the course material and simulation environments.

