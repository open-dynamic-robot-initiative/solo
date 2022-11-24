# Solo

Low level interface to control the solo robots.

### Installation

#### Download the package

We use `treep` to download the required packages. Make sure your ssh key is unlocked. Then


```bash
mkdir -p ~/devel
pip install treep  # This installs treep
cd ~/devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone SOLO
```

If you are using the Solo8 with TI boards, you have to get the SOLO_TI instead

```bash
mkdir -p ~/devel
pip install treep  # This installs treep
cd ~/devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone SOLO_TI
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

The API documentation of the current master branch is provided here:
https://open-dynamic-robot-initiative.github.io/solo

To build the API documentation yourself, please follow the steps [here](https://github.com/machines-in-motion/machines-in-motion.github.io/issues/4).

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2018-2021, New York University and Max Planck Gesellschaft.
