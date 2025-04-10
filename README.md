# mujoco dynamics docking (sdf version)

Welcome to the mujodo dynamics docking (sdf version) repository! 

This project is designed to provide a comprehensive solution for trajectory planning and dynamics simulation in robotics using the Mujoco simulator. It's built on top of the powerful Pinocchio library, which allows for efficient forward kinematics calculations.


## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Instruction](#instruction)
- [Usage](#usage)
- [Result](#result)
- [Trouble shooting](#trouble-shooting)
- [Contributing](#contributing)
- [License](#license)

## Introduction
This is a repository for mujoco dynamics simulation and compliance control.

## Features

- signed distance field (sdf)
- dynamics simulation
- compliance control

## Instruction
### 1. Model Files
1. iiwa14_dock.xml is an iiwa model with sdf linked at the end;
2. iiwa14_dock.urdf is the urdf version of iiwa14_dock.xml, used for pin's dynamics calculation;

### 2. Core Scripts
1. dynamics.py - Used to verify the dynamics of pin and mujoco
2. Relate_class.py - Contains related classes for the simulation,trajectory planning,inverse kinematics,dynamics control,etc;
3. task_dynamics_mujoco_control.py - Main simulation script for running the dynamics control

## Usage

To use this repository, follow these steps:

1. Clone the repository:

   ```bash
   git clone https://github.com/ming751/initial_docking_model.git
   ```

   ```bash
   cd initial_docking_model
   ```

2. Install the dependencies(if you use dymamic simulation, you need to install pinocchio,pinocchio must be installed in linux system):

   1) create an conda environment and activate it:
   ```bash
   conda create -n pin_mjcf python=3.10
   conda activate pin_mjcf
   ```
   
   2) install the pinocchio dependencies:
   ```bash
   pip install pin
   ```
   
   3) install other dependencies:
   ```bash
   pip install -r requirement.txt
   ```

3. Run the script:

   ### Main simulation script

   ```bash
   python task_dynamics_mujoco_control.py
   ```

## result
![docking error](demo/tracking_error.png)
📽️ [点击观看演示视频](demo/docking.mp4)



## Trouble shooting
How to render on headless device?
```bash
sudo apt update
sudo apt install libegl1 libegl-dev
export MUJOCO_GL=egl
```

## Contributing

We welcome contributions to the RM Robot project! If you would like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix:
   ```bash
   git checkout -b my-feature-branch
   ```
3. Make your changes and commit them:
   ```bash
   git commit -am 'Add new feature'
   ```
4. Push your changes to your fork:
   ```bash
   git push origin my-feature-branch
   ```
5. Create a pull request on GitHub.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.