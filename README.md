# Manipulation Driver
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/suddrey-qut/manipulation_driver.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/suddrey-qut/manipulation_driver/context:python)
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![Build Status](https://github.com/suddrey-qut/manipulation_driver/workflows/Build/badge.svg?branch=master)](https://github.com/suddrey-qut/manipulation_driver/actions?query=workflow%3ABuild)


## Usage

### Panda in Swift
```sh
rosrun manipulation_driver driver _robot:=roboticstoolbox.models.URDF.Panda _backend:=roboticstoolbox.backends.Swift
```

### UR5 in Swift
```sh
rosrun manipulation_driver driver _robot:=roboticstoolbox.models.URDF.UR5 _backend:=roboticstoolbox.backends.Swift
```

### Panda in PyPlot
```sh
rosrun manipulation_driver driver _robot:=roboticstoolbox.models.URDF.Panda _backend:=roboticstoolbox.backends.Swift
```
