# Dynamic_Interpolation_iLQR
This work appeared in the proceedings of ICRA 2023. This repository is out of date now, the new repository with additional functionality, documentation and usability is located [here](https://github.com/DMackRus/TrajOptKP).

## Video
- TODO link to video

## Installation Requirements
- MuJoCo V2.10(Included in this repository)
- Eigen V3.40

## Foreword
This repository is intended to work for the most part after downloading this repository as well as the [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page). Once downloaded main.cpp can be run to optimise and display different optimised trajectories. Currently there are three implemented tasks in this repository (Pendulum control task, reaching a desired configuration of a 7-DoF arm as well as pushing a cylinder along the ground to a desired location with the 7-DoF arm.)

## General repository layout
### MuJoCo Library
The MuJoCo library is currently version 2.10 and is included in this repository under "mujoco/include". 

### iLQR Implementation
My iLQR implementation written in c++ is found under "iLQR"

### Model files
General xml files for the three different tasks that can be loaded via MuJoCo can be found under "franka_emika". "Acrobot.xml" is the pendulum control task, "reaching.xml" is the reaching a configuration task and finally "object-pushing" is the final task.

### Model Translator files
Model translator files are the method in which the iLQR algorithm receives information it requires about the system so that it can optimise trajectories. The model translator file needs to be changed depending on the task. The current model translator file is found under "modelTranslator" and works for the three tasks as long as the correct #define variable is set depending on the desired task.

## Using this repository - As is
To use the repository for the tasks currently defined can be done quite easily by just changing certain #define variables in a few different files. The repository has different "operating modes" which are define in main.cpp at the top of the file by some #define statements. Currently, there are four operating modes, realistically the average user will only need the first mode for testing this repository, the modes are:
-   RUN_ILQR - Runs iLQR for a given task and given scene, once optimisation is done it will show the finalised trajectory on repeat in a window.
-   GENERATE_A_B - Generates A and B matrices via finite differencing for ten initial trajectories and saves them to a file
-   ILQR_DATA_COLLECTION - Runs the current iLQR optimisation method on a given task for the set number of trajectories and saves all the testing metrics to a file in the folder "testing_data"
-   MAKE_TESTING_DATA - Creates a set of starting and desired states randomly for a given task and saves them to a file.
-   MPC_TESTING - Uses the iLQR optimiser in a more MPC like method, rolling out segments of controls and re-planning as required. Until either the task is completed, or timeout occurs.

### Changing the task
To change the task currently being optimised you need to go into the "modelTranslator.h" and at the top of the file will be various commented out #define variables name "PENDULUM", "REACHING" AND "OBJECT_PUSHING". ONLY 1 of these should be uncommented otherwise the program won't load as it will instantiate multiple different versions of functions with identical names. Set the task you want active to be uncommented and leave the other two commented out.

After this, the correct task will be loaded, to change the scene you will need to specify a row number which will correspond to a csv file with different randomly created scenes that will be instantiated and solved by the iLQR optimiser.

## Authors
[David Russell](https://github.com/DMackRus) - el16dmcr@leeds.ac.uk
