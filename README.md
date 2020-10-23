# pybullet-planning (ss-pybullet)

A repository of PyBullet utility functions for robotic motion planning, manipulation planning, and task and motion planning (TAMP).
This repository was originally developed for the [PDDLStream](https://github.com/caelan/pddlstream) (previously named [STRIPStream](https://github.com/caelan/stripstream)) approach to TAMP.

<!--![Alt text](images/test.png?raw=true "Title")-->
<!--img src="images/pr2.png" height="300">&emsp;<img src="images/kuka.png" height="300"-->

<!-- ## PyBullet Planning -->

With the help of [Yijiang Huang](https://github.com/yijiangh), a stable and documented fork of **ss-pybullet** named [pybullet_planning](https://github.com/yijiangh/pybullet_planning) is available through [PyPI](https://pypi.org/project/pybullet-planning/).
However, new features will continue to be introduced first through **ss-pybullet**. 

## Citation

Caelan Reed Garrett. PyBullet Planning. https://pypi.org/project/pybullet-planning/. 2018.

## Installation

Install PyBullet on OS X or Linux using: 
```
$ pip install numpy pybullet
$ git clone --recurse-submodules https://github.com/caelan/ss-pybullet.git
$ cd ss-pybullet
$ git pull --recurse-submodules
```

## IKFast Compilation

We recommend using [IKFast](http://openrave.org/docs/0.8.2/openravepy/ikfast/), an analytical inverse kinematics solver, instead of PyBullet's damped least squares method solver.
IKFast bindings are included for the following robots.
To create IKFast bindings for a new robot, following the instructions in [ikfast_pybind](https://github.com/yijiangh/ikfast_pybind). 

<!-- https://pypi.org/project/ikfast-pybind/ -->

### Franka Panda

```
$ cd ss-pybullet/pybullet_tools/ikfast/franka_panda
$ python setup.py build
```

### MOVO

```
$ cd ss-pybullet/pybullet_tools/ikfast/movo
$ python setup.py -a {left,right}
```

### PR2

```
$ cd ss-pybullet/pybullet_tools/ikfast/pr2
$ python setup.py build
```

## Tests

1) Test pybullet - ```python -c 'import pybullet'```

## Examples

Examples:
* Kuka IIWA pick planning - ```$ python -m examples.test_kuka_pick```
* PR2 motion planning - ```$ python -m examples.test_pr2_motion```
* PR2 teleoperation - ```$ python -m examples.teleop_pr2```
* PR2 visibility - ```$ python -m examples.test_visibility```
* PR2 copying - ```$ python -m examples.test_clone```
* TurtleBot collisions - ```$ python -m examples.test_turtlebot```
* Franka Panda workspace planning - ```$ python -m examples.test_franka```
* MOVO random workspace planning - ```$ python -m examples.test_movo```
* Dropping beads - ```$ python -m examples.test_water```

<img src="images/turtlebot.png" height="150">&emsp;<img src="images/movo.png" height="150">

## PDDLStream Examples

See the following examples: https://github.com/caelan/pddlstream/tree/master/examples/pybullet

[<img src="https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg" height="200">](https://www.youtube.com/watch?v=3HJrkgIGK7c)
[<img src="https://img.youtube.com/vi/oWr6m12nXcM/0.jpg" height="200">](https://www.youtube.com/watch?v=oWr6m12nXcM)

## Forks

* https://github.com/yijiangh/pybullet_planning
* https://github.com/rachelholladay/pb_robot

## Gallery

* PDDLStream for TAMP - https://github.com/caelan/pddlstream
* Online TAMP - https://github.com/caelan/SS-Replan
* Automated Construction - https://github.com/caelan/pb-construction
* Learning + TAMP (LTAMP) - https://github.com/caelan/LTAMP

## PyBullet Resources

* PyPI - https://pypi.python.org/pypi/pybullet
* Quickstart - https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/
* Forum - https://pybullet.org/Bullet/phpBB3/
* Wordpress - https://pybullet.org/wordpress/
* Examples - https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples
* Bindings - https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/pybullet.c

## Bullet Resources

* GitHub - https://github.com/bulletphysics/bullet3
