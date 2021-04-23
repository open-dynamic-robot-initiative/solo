# Solo

Low level interface to control the solo robots.

### Installation

#### Download the package

We use `treep` to download the required packages. Make sure your ssh key is unlocked. Then


```
mkdir -p ~/devel
pip install treep  # This installs treep
cd ~/devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone SOLO
```

#### Build the package

We use [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
to build this package:

```
cd mkdir -p ~/devel/workspace
colcon build
```

### Usage

#### Demos/Examples

You find examples for how to use the code base in the `demos/` folder.

#### API documentation

To build the API documentation, please follow the steps [here](https://github.com/machines-in-motion/machines-in-motion.github.io/issues/4).

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2018-2021, New York University and Max Planck Gesellschaft.
