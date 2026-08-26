# Collection of Student Code of the Institut of Assembly Technology
This repository contains all code of the students working at the Institut of Assembly Technology of the Leibniz University Hannover.

## Add yourself to the overview table
1. Fork this repository by clicking on the `fork`-button: ![ForkButton](documentation/fork_button.png). Afterwards click `Create fork`.
2. Click on the `README.md`-file and then click on the edit-button: ![EditButton](documentation/edit_button.png) 
3. Add another line to the table at the bottom. (You can copy the line of the student above for the structure)
4. Column `Id`: Use the (planned) submission date of your thesis in the format yymmdd.
5. Column `Thesis title`: Insert the title of your thesis.
6. Column `Description`: Explain in one sentence what algorithm or code you developed in your thesis.
7. Type in a meaningful commit message and commit the changes.
8. Go to the chapter [How to add your code](## How to add your code) and perform the steps of one of the two methods.

## How to add your code
There are two methods for adding your code. The first is only using the Github-Webinterface. The second option is by cloning the repository and working on your computer. We advice to use the second method.

### First Method
1. Navigate into the `student_code`-folder.
2. Add a folder in `match_student_code/student_code` with a `README.md` file by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and select `Add new file`. 
3. Think of a name that summarizes your work/algorithm. This name is used in the next step.
4. Behind `match_student_code/student_code/` type `Id_NameOfStep10/README.md`. Insert the `Id` from the chapter [Add yourself to the overview table](## Add yourself to the overview table) and the name you thought of from `step 3`.
5. Navigate into the created folder `Id_NameOfStep3`.
6. Add all packages and additional files into the folder by clicking the `Add file`-button: ![AddFileButton](documentation/add_file_button.png) and then `Upload files`. Drag and drop all folders or files from the File Explorer into the box on the web interface to upload. (On Windows it is Windows Explorer on Ubuntu it is File Manager)
7. Type in a meaningful commit message and commit the changes.
8. Add a `README.md`-file in your folder (see `step 4`) and use the structure of the example-README-file in `000_ExampleFolder`. If applicable, create a setup.sh file to automate the installation of your code. An example setup.sh can be found [here](https://github.com/match-ROS/match_mobile_robotics/blob/noetic-devel/setup_full.sh).
9. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
10. Type in a meaningful commit message and commit the changes.
11. To finish, please jump to the chapter [Pull Request](### Pull Request) where you create a pull request to the main repository.

### Second Method
1. Clone the repository to your PC by executing the `git clone`-command with the forked repository.
2. Think of a name that summarizes your work/algorithm. This name is used in the next step.
3. Create a folder in `match_student_code/student_code` named `Id_NameOfStep2`. Insert the `Id` from the chapter [Add yourself to the overview table](## Add yourself to the overview table) and the name you thought of from `step 2`.
4. Add all packages and additional files into the folder, f.e. by drag and drop.
5. Add a `README.md`-file in your folder (see `step 3`) and use the structure of the example-README-file in `000_ExampleFolder`.
6. Fill out every necessary information in the copied structure and add any more details you want to describe or use your package.
7. Type in a meaningful commit message and commit the changes.
8. If not already done, push the changes to your forked repository.
9. To finish, please jump to the chapter [Pull Request](### Pull Request) where you create a pull request to the main repository.

### Pull Request
To merge your changes/added files into the official match-ROS repository, we need to create a Pull Request:
1. Navigate to the start page of your forked version of the `match_student_code` repository.
2. Check if the last commit message is by you. If not, you need to go into your forked version of the `match_student_code` repository.
3. Click on the `Pull Request`-button: ![PullRequestButton](documentation/pull_request_button.png) and then `New Pull Request` on the right side.
4. Compare two things with the following picture. Firstly, does the `base repository:` and `base:` match excactly. Secondly, is your forked version of the `match_student_code` repository listed behind `head repository` and is the branch selected that you commited your code to. 

![grafik](https://user-images.githubusercontent.com/50292612/211014212-b623642f-1ab7-4cd2-b9cb-03a260362e44.png)

5. Check if all changes are listed correctly. Did you add all necessary packages? Did you add all additional Matlab or Python scripts?
6. Create the pull request by clicking on the `Create Pull Request`-button.
7. Inform your supervisor about the newly created pull request.

## Content Overview
| Id | Thesis title | Description |
| --- | --- | --- |
| [131001](student_code/000_ExampleFolder/README.md) | Example Folder | This folder contains templates and examples how it should look like |
| [240101a](student_code/240101a_UltrasoundBasedLocalization/README.md) | Evaluation of an Ultrasound-based Indoor Localization System for Mobile Multi-robot Systme | In this work we developed a localization system based on the sensor fusion approach using the EKF|
| [240101b](student_code/240101b_SplinedVoronoiPlanner/README.md) | Planung und Glättung von Pfaden für Formationen nicht-holonomer mobiler Roboter | Path planning with voronoi diagrams and smoothing with quintic bezier-splines |
| [240101c](student_code/240101c_RedundancyRes/Readme.md) | Development of an Optimization Function for Redundancy Resolution while Avoiding Singularities for Redundant Mobile Manipulators | Optimization for the UR16 and MiR100/200 on basis of reduced gradient |
| [240101d](student_code/240101d_ips_sensor_fusion/README.md) | Entwicklung und Evaluierung von Konzepten zur Nutzung von Ultraschall Indoor Positioning Systemen in der mobilen Robotik | concept for fusing ultrasonic ips data with amcl and imu using kalman filters |
| [240101e](student_code/240101e_Recursive-Least-Squares-Algorithm/README.md) | Entwicklung eines Algorithmus zur Schätzung der dynamischen Parameter unbekannter Objekte in einem Multirobotersystem | In this work we developed an Recursive-Least-Squares Algorithm to estimate the dynamic parameters of an unknown object |
| [240101f](student_code/240101f_FormationLayer/README.md) | Conception and implementation of an algorithm for the dynamic generation of the costmap of a multi-robot-system | A plugin costmap layer for a multi-robot-system |
| [240101g](student_code/240101g_FormationBuilder/README.md) | Entwicklung einer skalierbaren Multiagenten-Pfadplanung zur Formationsbildung für nicht-holonome mobile Roboter | Multi-agent path planning algorithm for transitioning robots into a formation.  |
| [240410](student_code/240410_bezierSplines/README.md) | Evaluierung von Bahnglättungsalgorithmen für Formationen bestehend aus nicht-holonomen mobilen Robotern | An algorithm that generates bezier Splines on the basis of any path planner for the use of formations of non-holonomic mobile robots |
| [241107](student_code/241107_AMCL_Object_Geometries/README.md) | Improving Localization Accuracy Using AMCL by Integrating Object Geometries | Method to insert object geometries into RVIZ and scripts for collecting data to calculate AMCL accuracy. |
| [241216](student_code/241216_stabilizing_rough_terrain/README.md) | Auslegung eines Echtzeit-Erfassungssystems von Bodenunebenheiten mit synchronisierter Kompensation des Endeffektors für mobile Roboter | Entwicklung eines Stabilisierungsalgorithmusses für den Endeffektor bei Befahren einer Bodenunebenheit |
| [250204](student_code/250204_Cable_detection_for_LocalPP/README.md) | Development of a Visual-Based Local Path Planning Algorithm for Cable Avoidance | Algorithm for real-time Cable Detection and Pose Estimation to be integrated with Local Costmap |
| [250205](student_code/250205_MultiCableRobotPlanner/README.md) | Entwicklung eines globalen Bahnplanungsalgorithmus für ein kabelgebundenes Multi-Roboter-System | In this thesis a global path planning algorithm for a tangled multi-robot-system was developed. |
| [250318](student_code/250318_LIO-SAM_MID360_Simulation/README.md) | Development of a localization algorithm for mobile manipulators in additive manufacturing | Implemente and evaluate LIO-SAM and AMCL localization algorithms using a Livox MID360-equipped robot in a simulation environment. |
| [250408](student_code/250408_Dynamic_Model_for_mobile_robots/README.md) | Entwicklung eines dynamischen Modells zur flexiblen Implementierung auf mobilen Robotersystemen | Dynamische Modellierung von mobilen Robotern, wobei das Modell flexibel auf verschiedenste Roboter übertragen werden kann |
| [250428](student_code/250428_Scan_to_map_localization_Mid360_Simulation/README.md) | Development and implementation of a concept for localization using 3D-LiDAR | Algorithm for scan-to-map based 3D real-time localization |
| [250429](student_code/250429_real_time_stabilization/README.md) | Real-Time Compensation of Ground Irregularities for Mobile Robot in Construction Additive Manufacturing | Algorithm to stabilize the nozzle for additive manufacturing purposes using IMU-based inclination data during movement on uneven terrain |
| [251127](student_code/251127_print_texture_localization/README.md) | Concept development for robot localization based on an object to be manufactured | Concept for texture-based localization approach for mobile manipulator based on the initial layers of the printed object, using image descriptors and ICP refinement|
| [260722](student_code/260722_UAVViewPlanning/README.md) | Entwicklung eines Algorithmus zur optimalen Abtastung von großskaligen Messobjekten mittels UAV-basierter Sensorik | Offline View Planning für ein UAV-getragenes Streifenlichtsystem: Set Cover über eine geraycastete Sichtbarkeitsmatrix, 4-DoF-Posen, Registrierungsgraph und TSP-Sequenzierung |
