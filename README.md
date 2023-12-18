# Magician defects detection


## Installation

### Repository download

To start this repository you can simply download through https:

``` bash
git clone https://github.com/BoroBonez/Magician_Defects_Detection.git
```

You can also install the repository on a given path and with a given name with the following command:
``` bash
git clone https://github.com/BoroBonez/Magician_Defects_Detection.git /path/to/ros_ws/src
```

that will generate the following folder tree:
```
/path/to/ros_ws/src
├── computing_point_cloud
├── display_urdf
├── optitrack2rviz
├── optitrack_interface
├── realsense-ros
├── zed-ros2-wrapper
└── README.md

# Structure as of 18-12-2023
```

**Note:** if you plan on working with this repository, it is recomended to clone the
repository through SSH authentication. The previous commanded are respectively

``` bash
git clone git@github.com:BoroBonez/Magician_Defects_Detection.git
```

and
``` bash
git clone git@github.com:BoroBonez/Magician_Defects_Detection.git /path/to/ros_ws/src
```

A simple guide on how to setup a SSH key for authentication with Github can be found
[here](https://idra-lab.github.io/idra-shared-knowledge/02_linux/04_ssh/00_index.html).

### External dependencies
Some dependencies are imported as git submodules that can be pulled with the following
command:

``` bash
git submodule update --init --recursive
```
