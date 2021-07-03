# Awesome Robotics

A list of important content related to Mobile Robotics.

## Contents

- **[News](#news)**
- **[Books](#books)**
- **[Courses](#courses)**
- **[SLAM](#slam)**
- **[Planning](#planning)**
- **[Control](#control)**
- **[Survey Papers](#survey-papers)**
- **[Software](#software)**
- **[Sensors](#sensors)**
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

## Books

- [Probabilistic Robotics](https://dl.acm.org/doi/10.5555/1121596) | 2005
- [Introduction to Autonomous Mobile Robots](https://www.amazon.com/Introduction-Autonomous-Mobile-Intelligent-Robotics/dp/0262015358/)
- [Introduction to Autonomous Robots](https://github.com/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots)
- [Springer Handbook of Robotics](https://www.amazon.com/Springer-Handbook-Robotics-Handbooks/dp/3319325507/)
- [Where am I? Sensors and Methods for Mobile Robot Positioning](http://citeseerx.ist.psu.edu/viewdoc/citations;jsessionid=845D198BC9C1C6D04D04ED6D9B8280AB?doi=10.1.1.228.903)
- [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/index.php/LynchAndPark)

## Courses

- [GraphSLAM tutorials code](https://github.com/HeYijia/GraphSLAM_tutorials_code)
- [The what and why of GRAPH SLAM!](https://garimanishad.medium.com/everything-you-need-to-know-about-graph-slam-7f6f567f1a31)
- [SLAM Tutorial@ICRA 2016](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)
- [Autonomous Mobile Robots](https://courses.edx.org/courses/ETHx/AMRx/1T2014/info)
- [Introduction to Mobile Robotics - UniFreiburg](http://ais.informatik.uni-freiburg.de/teaching/ss16/robotics/)
- [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

## SLAM

### Monocular
- [ORB_SLAM](https://github.com/raulmur/ORB_SLAM) | A Versatile and Accurate Monocular SLAM
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

## Planning

- [Intro to A*](https://www.redblobgames.com/pathfinding/a-star/introduction.html)

## Control

- [FastPID](https://github.com/mike-matera/FastPID)

## Survey Papers

- [Robotic Mapping: A Survey](http://robots.stanford.edu/papers/thrun.mapping-tr.pdf) (2002)
- [Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age](https://arxiv.org/abs/1606.05830) (2016)

## Software

### Frameworks

- [ROS](https://www.ros.org/) | Robot Operating System
- [ROS 2](https://index.ros.org/doc/ros2/) | Robot Operating System 2
- [YARP](http://www.yarp.it/) | Yet Another Robot Platform
- [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto) | An open-source software stack based on ROS 2 for self​-driving technology
- [OpenPilot](https://github.com/commaai/openpilot) | An open source driver assistance system
- [Apollo](https://github.com/ApolloAuto/apollo) | Autonomous Driving Solution

### ROS Packages

- [robot_localization](http://wiki.ros.org/robot_localization)
- [imu_filter_madgwick](http://wiki.ros.org/imu_filter_madgwick)

### Simulators

- [Gazebo](http://gazebosim.org/)
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- [SVL](https://www.svlsimulator.com/)
- [Unity](https://unity.com/solutions/automotive-transportation-manufacturing/robotics)
- [CARLA](http://carla.org/)
- [Webots](https://cyberbotics.com/)
- [AirSim](https://github.com/microsoft/AirSim)

### Libraries

- [OpenCV](http://opencv.org/) | Computer Vision library
- [PCL](https://pointclouds.org/) | Point Cloud Library
- [Eigen](http://eigen.tuxfamily.org) | C++ template library for linear algebra: matrices, vectors and numerical solvers
- [MRPT](https://www.mrpt.org/) | Mobile Robot Programming Toolkit
- [OctoMap](https://github.com/OctoMap/octomap) | Efficient Probabilistic 3D Mapping Framework Based on Octrees
- [RansacLib](https://github.com/tsattler/RansacLib) | RANSAC Implementation
- [ICP](https://github.com/ClayFlannigan/icp) | Iterative Closest Point
- [libpointmatcher](https://libpointmatcher.readthedocs.io/en/latest/) | Iterative Closest Point implementation
- [OMPL](https://github.com/ompl/ompl)- The Open Motion Planning Library
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | Various robotics algorithms

#### Optimization

- [Ceres Solver](https://github.com/ceres-solver/ceres-solver) | Nonlinear optimization library
- [g2o](https://github.com/RainerKuemmerle/g2o) | C++ framework for optimizing graph-based nonlinear error functions
- [CasADi](https://github.com/casadi/casadi) | A tool for nonlinear optimization and algorithmic differentiation

#### Nearest Neighbor Search

- [FLANN](https://github.com/flann-lib/flann) | Fast Library for Approximate Nearest Neighbors
- [nanoflann](https://github.com/jlblancoc/nanoflann) | C++ library for building KD-Trees
## Sensors

### Lidars

- [Velodyne](https://velodynelidar.com/)
- [Livox](https://www.livoxtech.com/)
- [SICK](https://www.sick.com/ag/en/)
- [Hokuyo](https://www.hokuyo-aut.jp/)

### RGB-D sensors

- [Kinect](https://electronics.howstuffworks.com/microsoft-kinect2.htm)

## Datasets

- [Awesome SLAM Datasets](https://github.com/youngguncho/awesome-slam-datasets)
- [TUM University - RGBD data](http://vision.in.tum.de/data/datasets/rgbd-dataset)
- [KITTI Vision](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- [Gazebo models](https://github.com/osrf/gazebo_models)
- [NASA 3D models](https://github.com/nasa/NASA-3D-Resources)

## Journals and conferences

### Journals

- [Autonomous Robots](http://www.springer.com/engineering/robotics/journal/10514)
- [Robotics and Autonomous Systems](https://www.journals.elsevier.com/robotics-and-autonomous-systems)
- [International Journal of Robotics Research (IJRR)](https://journals.sagepub.com/home/ijr)
- [Journal of Field Robotics (JFR)](https://onlinelibrary.wiley.com/journal/15564967)
- [Journal of Intelligent & Robotic Systems](http://www.springer.com/engineering/robotics/journal/10846)
- [Robotics & Automation Magazine (RAM)](https://www.ieee-ras.org/publications/ram)
- [Transactions on Robotics (T-RO)](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8860)
- [Robotica](https://www.cambridge.org/core/journals/robotica)
- [Robotics and Automation Letters (RA-L)](https://www.ieee-ras.org/publications/ra-l)
- [Robotics and Computer-Integrated Manufacturing](https://www.journals.elsevier.com/robotics-and-computer-integrated-manufacturing)
- [IEEE Transactions on Robotics](http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8860)
- [IEEE Robotics & Automation Magazine](http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=100)
- [Frontiers in Robotics and AI](http://journal.frontiersin.org/journal/robotics-and-ai)
- [Mechatronics](http://www.journals.elsevier.com/mechatronics)
- [Robotics and Computer-Integrated Manufacturing](http://www.journals.elsevier.com/robotics-and-computer-integrated-manufacturing)
- [Bioinspiration & Biomimetics](http://iopscience.iop.org/journal/1748-3190)
- [IEEE/ASME Transactions on Mechatronics](http://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=3516)
- [International Journal of Social Robotics](http://www.springer.com/engineering/robotics/journal/12369)
- [Journal of Automation, Mobile Robotics and Intelligent Systems](https://www.jamris.org)

### Conferences

- [IEEE International Conference on Robotics and Automation (ICRA)](http://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra)
- [Robotics: Science and Systems Conference (RSS)](http://www.roboticsconference.org/)
- [IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)](http://www.iros.org/)
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
- [ABB Robotics](http://new.abb.com/products/robotics) the manufacturer of industrial robots
- [KUKA Robotics](http://www.kuka-robotics.com/en/) major manufacturer of industrial robots
- [FANUC](http://www.fanucamerica.com/) industrial robots manufacturer
- [Soft Robotics](https://www.softroboticsinc.com/)
- [The construct sim](http://www.theconstructsim.com/)  A cloud based tool for building robot simulations
- [PAL Robotics](http://pal-robotics.com)
- [iRobot](http://www.irobot.com/) manufacturer of the Roomba
- [Aldebaran Robotics](https://www.aldebaran.com/en) manufacturer of the Nao robot
- [Rethink Robotics](http://www.rethinkrobotics.com/) creator of Baxter robot
- [DJI](http://www.dji.com/)
- [Halodi Robotics](https://www.halodi.com/)

## Miscellaneous

### Robots

- [Turtlebot](https://www.turtlebot.com/)
- [PR2](http://wiki.ros.org/Robots/PR2)

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
- [awesome-robotic-tooling](https://github.com/Ly0n/awesome-robotic-tooling)
- [awesome-cpp](https://github.com/fffaraz/awesome-cpp)
- [awesome-mobile-robotics](https://github.com/mathiasmantelli/awesome-mobile-robotics)
- [awesome-lidar](https://github.com/szenergy/awesome-lidar)
- [awesome-self-driving](https://github.com/Sid1057/awesome-self-driving)