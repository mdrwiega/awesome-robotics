# Awesome Robotics

A list of important content related to Mobile Robotics.
Check also the [robotics glossary of acronyms](./glossary.md).

## Contents

- **[News](#news)**
- **[Books](#books)**
- **[Survey Papers](#survey-papers)**
- **[Courses](#courses)**
- **[Perception](#perception)**
- **[SLAM](#slam)**
- **[Planning](#planning)**
- **[Control](#control)**
- **[AI](#ai)**
- **[Software](#software)**
- **[Hardware](#hardware)**
- **[Electronics](#electronics)**
- **[Mechanics](#mechanics)**
- **[Datasets](#datasets)**
- **[Journals and conferences](#journals-and-conferences)**
- **[Companies](#companies)**
- **[Miscellaneous](#miscellaneous)**

## News

- [IEEE Spectrum Robotics](https://spectrum.ieee.org/robotics)
- [The Robot Report](https://www.therobotreport.com/)
- [MIT Robotics](https://news.mit.edu/topic/robotics)
- [Robotics And Automation News](https://roboticsandautomationnews.com/)
- [Robotics Business Review](https://www.roboticsbusinessreview.com/)
- [Reddit/robotics](https://www.reddit.com/r/robotics/)

## Books

- [Probabilistic Robotics](https://dl.acm.org/doi/10.5555/1121596) | S. Thrun et al. | 2005
- [Introduction to Autonomous Mobile Robots](https://www.amazon.com/Introduction-Autonomous-Mobile-Intelligent-Robotics/dp/0262015358/) | R. Sigwart et al. | 2004
- [Introduction to Autonomous Robots](https://github.com/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots) | N. Correll et al. | 2022
- [Springer Handbook of Robotics](https://link.springer.com/book/10.1007/978-3-540-30301-5) | 2008
- [Where am I? Sensors and Methods for Mobile Robot Positioning](http://www-personal.umich.edu/~johannb/Papers/pos96rep.pdf) | J. Borenstein et al. | 1996
- [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/index.php/LynchAndPark) | K. Lynch, F. Park | 2017
- [Robotics, Vision, and Control: Fundamental Algorithms in MATLAB](https://link.springer.com/book/10.1007/978-3-319-54413-7) | P. Corke | 2017

## Survey Papers

- [Robotic Mapping: A Survey](http://robots.stanford.edu/papers/thrun.mapping-tr.pdf) (2002)
- [Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age](https://arxiv.org/abs/1606.05830) (2016)

## Courses

- [GraphSLAM tutorials code](https://github.com/HeYijia/GraphSLAM_tutorials_code)
- [The what and why of GRAPH SLAM!](https://garimanishad.medium.com/everything-you-need-to-know-about-graph-slam-7f6f567f1a31)
- [SLAM Tutorial@ICRA 2016](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)
- [Autonomous Mobile Robots](https://courses.edx.org/courses/ETHx/AMRx/1T2014/info)
- [Introduction to Mobile Robotics - UniFreiburg](http://ais.informatik.uni-freiburg.de/teaching/ss16/robotics/)
- [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
- [Multidimensional Kalman-Filter](https://github.com/balzer82/Kalman)
- [How a Kalman filter works in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
- [Self-Driving Cars with ROS and Autoware](https://www.apex.ai/autoware-course)
- [edX Autonomous Mobile Robots](https://www.edx.org/course/autonomous-mobile-robots)
- [edX Self-Driving Cars with Duckietown](https://www.edx.org/course/self-driving-cars-with-duckietown)

## Perception

### Computer Vision

- [OpenCV](http://opencv.org/) | Open-source Computer Vision library focused on real-time vision
- [DeepFace](https://github.com/serengil/deepface/tree/master/deepface) | Open-source face recognition Python library

### 3D Data Processing

- [PCL](https://pointclouds.org/) | Point Cloud Library
- [Open3D](http://www.open3d.org/docs/release/) | Library for 3D data processing
- [ICP](https://github.com/ClayFlannigan/icp) | Iterative Closest Point
- [libpointmatcher](https://libpointmatcher.readthedocs.io/en/latest/) | Open-source implementation of the Iterative Closest Point
- [OctoMap](https://github.com/OctoMap/octomap) | Efficient Probabilistic 3D Mapping Framework Based on Octrees
- [RansacLib](https://github.com/tsattler/RansacLib) | RANSAC Implementation

## SLAM

### Monocular

- [ORB_SLAM](https://github.com/raulmur/ORB_SLAM) | A Versatile and Accurate Monocular SLAM
- [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | Real-time SLAM able to perform Visual, Visual-Inertial and Multi-Map SLAM with monocular, stereo and RGB-D cameras
- [LSD-SLAM](https://github.com/tum-vision/lsd_slam) | Large-Scale Direct Monocular SLAM
- [SVO](https://github.com/uzh-rpg/rpg_svo) | Semi-Direct Monocular Visual Odometry
- [DSO](https://github.com/JakobEngel/dso) | Direct Sparse Odometry
- [LDSO](https://github.com/tum-vision/LDSO) | Direct Sparse Odometry with Loop Closure
- [PTAM](https://github.com/Oxford-PTAM/PTAM-GPL) | Parallel Tracking and Mapping
- [LPVO](https://github.com/PyojinKim/LPVO) | Line and Plane based Visual Odometry
- [LCSD_SLAM](https://github.com/sunghoon031/LCSD_SLAM) | Loosely-Coupled Semi-Direct Monocular SLAM

### Stereo

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [S-PTAM](https://github.com/lrse/sptam) | Stereo Parallel Tracking and Mapping

### RGB-D

- [DVO: Dense Visual Odometry](https://github.com/tum-vision/dvo_slam)
- [RGBD-SLAM](http://felixendres.github.io/rgbdslam_v2/)

### Other

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Graph-SLAM](https://www.mrpt.org/Graph-SLAM_maps)
- [Hector SLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
- [RBPF-SLAM](https://www.mrpt.org/tutorials/slam-algorithms/rbpf-slam_algorithms/)
- [ElasticFusion](https://github.com/mp3guy/ElasticFusion)
- [LOAM-Livox](https://github.com/hku-mars/loam_livox)
- [GMapping](https://openslam-org.github.io/gmapping.html) | SLAM based on Rao-Blackwellized particle filter
- [OpenSLAM](http://openslam.org/)

## Optimization

- [CasADi](https://web.casadi.org/)
- [ACADO Toolkit](https://acado.github.io/)
- [Ceres Solver](http://ceres-solver.org/)
- [g2o](https://github.com/RainerKuemmerle/g2o)

## Planning

- [Intro to A*](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [Path finding](https://qiao.github.io/PathFinding.js/visual/) | Path finding algorithms visualization
- [OMPL](https://github.com/ompl/ompl) | The Open Motion Planning Library
- [MoveIt](https://moveit.ros.org/) | Motion planning framework

## Control

- [FastPID](https://github.com/mike-matera/FastPID)

## AI

- [YOLOv5](https://github.com/ultralytics/yolov5)

## Software

### Tools

- [3D Rotation Converter](http://www.andre-gaschler.com/rotationconverter/)

### Frameworks

- [ROS](https://www.ros.org/) | Robot Operating System
- [ROS 2](https://index.ros.org/doc/ros2/) | Robot Operating System 2
- [YARP](http://www.yarp.it/) | Yet Another Robot Platform
- [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto) | An open-source software stack based on ROS 2 for self​-driving technology
- [OpenPilot](https://github.com/commaai/openpilot) | An open source driver assistance system
- [Apollo](https://github.com/ApolloAuto/apollo) | Autonomous Driving Solution

### ROS 2 Packages

- [geometry2](https://github.com/ros2/geometry2)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [cartographer](https://github.com/ros2/cartographer)
- [navigation2](https://github.com/ros-planning/navigation2)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [slam_gmappiong](https://github.com/Project-MANAS/slam_gmapping)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools)
- [moveit2](https://github.com/ros-planning/moveit2)

### Simulation

- [Ignition Gazebo](https://github.com/gazebosim/gz-sim) - successor of Gazebo
- [Gazebo](http://gazebosim.org/)
- [CoppeliaSim](https://www.coppeliarobotics.com/) | successor of V-REP
- [SVL](https://www.svlsimulator.com/)
- [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics)
- [CARLA](http://carla.org/)
- [Webots](https://cyberbotics.com/)
- [AirSim](https://github.com/microsoft/AirSim)
- [Player/Stage](http://playerstage.sourceforge.net/)
- [O3DE (Open 3D Engine)](https://www.o3de.org/)
- [NVIDIA Isaac](https://developer.nvidia.com/isaac-sdk)
- [MATLAB/Simulink](https://se.mathworks.com/)
- [Scilab/Xcos](https://www.scilab.org/) | Open-source alternative to Matlab and Simulink
- [Octave](https://octave.org/) | Another open-source alternative to Matlab

### Libraries

- [Eigen](http://eigen.tuxfamily.org) | C++ template library for linear algebra: matrices, vectors and numerical solvers
- [MRPT](https://www.mrpt.org/) | Mobile Robot Programming Toolkit
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | Various robotics algorithms
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)

#### Optimization

- [Ceres Solver](https://github.com/ceres-solver/ceres-solver) | Nonlinear optimization library
- [g2o](https://github.com/RainerKuemmerle/g2o) | C++ framework for optimizing graph-based nonlinear error functions
- [CasADi](https://github.com/casadi/casadi) | A tool for nonlinear optimization and algorithmic differentiation

#### Nearest Neighbor Search

- [FLANN](https://github.com/flann-lib/flann) | Fast Library for Approximate Nearest Neighbors
- [nanoflann](https://github.com/jlblancoc/nanoflann) | C++ library for building KD-Trees

## Hardware

### Sensors

#### Lidars

- [Velodyne](https://velodynelidar.com/)
- [Livox](https://www.livoxtech.com/)
- [SICK](https://www.sick.com/ag/en/)
- [Hokuyo](https://www.hokuyo-aut.jp/)
- [Ouster](https://ouster.com/)
- [Hesai](https://www.hesaitech.com/)

#### RGB-D sensors

- [Microsoft Kinect 2.0](https://electronics.howstuffworks.com/microsoft-kinect2.htm) | Time of flight
- [Microsoft Azure Kinect DK](https://azure.microsoft.com/en-us/products/kinect-dk/#overview) | Time of flight
- [Structure Core/Sensor Pro](https://structure.io/)
- [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) | Active IR Stereo
- [Intel RealSense D405](https://www.intelrealsense.com/depth-camera-d405/) | Stereo Camera
- [StereoLabs ZED 2](https://www.stereolabs.com/zed-2/) | Stereo camera with neural networks
- [Asus Xtion Pro Live](https://www.asus.com/me-en/supportonly/xtion%20pro%20live/helpdesk_knowledge/) | Structured light
- [Optonic Enseno N35-606-16-BL](https://www.optonic.com/en/brands/ensenso/?id=N35-606-16-BL) | Structured light
- [Orbec Astra Mini](https://orbbec3d.com/index/Product/info.html?cate=38&id=13) | Structured light

#### IMU

- [Bosch IMUs](https://www.bosch-sensortec.com/products/motion-sensors/imus/)
- [Phidgets](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)
- [Advanced Navigation Orientus](https://www.advancednavigation.com/imu-ahrs/mems-imu/orientus/)

## Electronics

- [KiCad EDA](https://www.kicad.org/) | Open-source tool for PCB design

## Mechanics

- [FreeCAD](https://github.com/FreeCAD/FreeCAD) | Open-source 3D modeling tool

## Datasets

- [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets)
- [TUM University - RGBD data](http://vision.in.tum.de/data/datasets/rgbd-dataset)
- [KITTI Vision](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- [Gazebo models](https://github.com/osrf/gazebo_models)
- [NASA 3D models](https://github.com/nasa/NASA-3D-Resources)

## Journals and conferences

### Journals

- [Autonomous Robots](http://www.springer.com/engineering/robotics/journal/10514) | *Springer*
- [Journal of Intelligent & Robotic Systems](http://www.springer.com/engineering/robotics/journal/10846) | *Springer*
- [International Journal of Social Robotics](http://www.springer.com/engineering/robotics/journal/12369) | *Springer*
- [RAM - Robotics & Automation Magazine](https://www.ieee-ras.org/publications/ram) | *IEEE*
- [Transactions on Robotics (T-RO)](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8860) | *IEEE*
- [RA-L - Robotics and Automation Letters](https://www.ieee-ras.org/publications/ra-l) | *IEEE*
- [IEEE/ASME Transactions on Mechatronics](http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=3516) | *IEEE*
- [IJRR - International Journal of Robotics Research](https://journals.sagepub.com/home/ijr) | *Sage*
- [IJARS - International Journal of Advanced Robotics Systems](https://journals.sagepub.com/home/arx) | *Sage*
- [JFR - Journal of Field Robotics](https://onlinelibrary.wiley.com/journal/15564967) | *Wiley*
- [Robotics and Autonomous Systems](https://www.journals.elsevier.com/robotics-and-autonomous-systems) | *Elsevier*
- [Robotics and Computer-Integrated Manufacturing](https://www.journals.elsevier.com/robotics-and-computer-integrated-manufacturing) | *Elsevier*
- [Mechatronics](http://www.journals.elsevier.com/mechatronics) | *Elsevier*
- [Robotics and Computer-Integrated Manufacturing](http://www.journals.elsevier.com/robotics-and-computer-integrated-manufacturing) | *Elsevier*
- [Robotica](https://www.cambridge.org/core/journals/robotica) | *Cambridge*
- [Frontiers in Robotics and AI](http://journal.frontiersin.org/journal/robotics-and-ai)
- [Journal of Automation, Mobile Robotics and Intelligent Systems](https://www.jamris.org)
- [Sensors](https://www.mdpi.com/journal/sensors) | *MDPI*
- [JOSS - Journal of Open Source Software](http://joss.theoj.org/)
- [JAMRIS - Journal of Automation, Mobile Robotics & Intelligent Systems](http://www.jamris.org)
- [International Journal of Robotics](http://www.hindawi.com/journals/jr/)

### Conferences

- [ICRA - IEEE International Conference on Robotics and Automation](http://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra)
- [RSS - Robotics: Science and Systems Conference](http://www.roboticsconference.org/)
- [IROS - IEEE/RSJ International Conference on Intelligent Robots and Systems](http://www.iros.org/)
- [International Symposium of Robotic Research (ISRR)](http://ifrr.org/isrr.php)
- [International Symposium of Experimental Robotics (ISER)](http://ifrr.org/iser.php)
- [Robotica](http://www.ieee-ras.org/conferences-workshops/technically-co-sponsored/robotica)
- [ACM/IEEE International Conference on Human Robot Interaction (HRI)](http://ieeexplore.ieee.org/xpl/conhome.jsp?punumber=1040036)
- [International Conference on Methods and Models in Automation and Robotics (MMAR)](http://mmar.edu.pl/)
- [ROSCon](https://roscon.ros.org/)

## Companies

- [Boston Dynamics](http://www.bostondynamics.com/)
- [Waymo](https://waymo.com) | Autonomous Driving car company from Google
- [Fetch Robotics](http://www.fetchrobotics.com/)
- [AutonomousStuff](https://autonomoustuff.com/)
- [Aptiv](https://www.aptiv.com/)
- [ABB Robotics](http://new.abb.com/products/robotics) | The manufacturer of industrial robots
- [KUKA Robotics](http://www.kuka-robotics.com/en/) | The manufacturer of industrial robots
- [FANUC](http://www.fanucamerica.com/) | Industrial robots manufacturer
- [Soft Robotics](https://www.softroboticsinc.com/)
- [The construct sim](http://www.theconstructsim.com/) | A cloud based tool for building robot simulations
- [PAL Robotics](http://pal-robotics.com)
- [iRobot](http://www.irobot.com/) | Manufacturer of the Roomba
- [Aldebaran Robotics](https://www.aldebaran.com/en) | Manufacturer of the Nao robot
- [Rethink Robotics](http://www.rethinkrobotics.com/) | Creator of Baxter robot
- [DJI](http://www.dji.com/)
- [Halodi Robotics](https://www.halodi.com/)
- [Nuro](https://www.nuro.ai/) - autonomous driving
- [Diligent Robotics](https://www.diligentrobots.com/)
- [Magazino](https://www.magazino.eu/?lang=en)
- [Accrea](http://engineering.accrea.com/)
- [Universal Robots](https://www.universal-robots.com/)
- [X](https://x.company/)
- [Small Robot Company](https://www.smallrobotcompany.com/)
- [Optimus Ride](https://www.optimusride.com/) | self-driving vehicles
- [Amazon Robotics](https://builtin.com/company/amazon)
- [UiPath](https://www.uipath.com/)
- [Accuray](https://www.accuray.com/) | Healthcare
- [Intuitive Surgical](https://www.intuitive.com/en-us) | Healthcare
- [Da Vinci Surgery](https://www.davincisurgery.com/) | Surgery robots

## Miscellaneous

### Robots

- [Turtlebot](https://www.turtlebot.com/)
- [PR2](http://wiki.ros.org/Robots/PR2)
- [Atlas](https://www.bostondynamics.com/atlas)
- [Husarion](https://husarion.com/)

### Projects

- [Rovina](http://www.rovina-project.eu/) (EU)
- [Mars 2020](https://mars.nasa.gov/mars2020/) (NASA)

### Other lists

[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

- [kiloreux/awesome-robotics](https://github.com/kiloreux/awesome-robotics)
- [kanster/awesome-slam](https://github.com/kanster/awesome-slam)
- [SilenceOverflow/Awesome-SLAM](https://github.com/SilenceOverflow/Awesome-SLAM)
- [SilenceOverflow/Awesome-SLAM-Papers](https://github.com/SilenceOverflow/Awesome-SLAM-Papers)
- [tzutalin/awesome-visual-slam](https://github.com/tzutalin/awesome-visual-slam)
- [awesome-SLAM-list](https://github.com/OpenSLAM/awesome-SLAM-list)
- [ahundt/awesome-robotics](https://github.com/ahundt/awesome-robotics)
- [awesome-ros2](https://github.com/fkromer/awesome-ros2)
- [awesome-robotics-libraries](https://github.com/jslee02/awesome-robotics-libraries)
- [awesome-computer-vision](https://github.com/jbhuang0604/awesome-computer-vision)
- [robotics-coursework](https://github.com/mithi/robotics-coursework)
- [awesome-robotic-tooling](https://github.com/protontypes/awesome-robotic-tooling)
- [awesome-cpp](https://github.com/fffaraz/awesome-cpp)
- [awesome-mobile-robotics](https://github.com/mathiasmantelli/awesome-mobile-robotics)
- [awesome-lidar](https://github.com/szenergy/awesome-lidar)
- [awesome-self-driving](https://github.com/Sid1057/awesome-self-driving)