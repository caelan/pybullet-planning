# ss-pybullet
STRIPStream + pybullet

<!--![Alt text](images/test.png?raw=true "Title")-->
<!--img src="images/pr2.png" height="300">&emsp;<img src="images/kuka.png" height="300"-->

[![Kuka IIWA](https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg)](https://www.youtube.com/watch?v=3HJrkgIGK7c)
[![PR2](https://img.youtube.com/vi/oWr6m12nXcM/0.jpg)](https://www.youtube.com/watch?v=oWr6m12nXcM)

## Installation

Install PyBullet on OS X or Linux using: 
```
pip install pybullet
```

## Tests

1) Test pybullet - ```python -c 'import pybullet'```
2) Test STRIPStream - ```python -c 'import ss'```

## Manipulation Examples

Examples:
* Kuka IIWA pick planning - ```python test_kuka_pick.py```
* PR2 motion planning - ```python test_pr2_motion.py```

## STRIPStream Examples

Add the following repository to your PYTHONPATH: https://github.com/caelan/ss

Examples:
* Kuka IIWA task and motion planning - ```python -m test_ss.test_kuka_ss```
* PR2 task and motion planning - ```python -m test_ss.test_pr2_ss```

## PDDLStream Examples

See the following examples: https://github.com/caelan/pddlstream/tree/master/examples/pybullet

## PyBullet Resources
* pip - https://pypi.python.org/pypi/pybullet
* Quickstart - https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/
* Forum - https://pybullet.org/Bullet/phpBB3/
* Wordpress - https://pybullet.org/wordpress/
* Examples - https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples

## Bullet Resources
* Github - https://github.com/bulletphysics/bullet3
