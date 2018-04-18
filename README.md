# ss-pybullet
STRIPStream + pybullet

<!--![Alt text](images/test.png?raw=true "Title")-->
<img src="images/pr2.png" height="300">&emsp;<img src="images/kuka.png" height="300">


## Installation
Install PyBullet on OSX or Linux using: 
```
pip install pybullet
```

Clone the following repositories and add them to your PYTHONPATH:
1) https://github.com/caelan/motion-planners
2) https://github.com/caelan/pddlstream

## Tests

1) Test pybullet - ```python -c 'import pybullet'```
2) Test motion-planners - ```python -c 'import motion_planners'```
3) Test STRIPStream - ```python -c 'import ss'```

## Examples

Examples without STRIPStream:
* PR2 motion planning - ```python test_motion.py```

Examples using STRIPStream:
* Kuka IIWA task and motion planning - ```python test_kuka.py```
* PR2 task and motion planning - ```python test_ss.py```

## PyBullet Resources
* pip - https://pypi.python.org/pypi/pybullet
* Quickstart - https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/
* Forum - https://pybullet.org/Bullet/phpBB3/
* Wordpress - https://pybullet.org/wordpress/
* Examples - https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples

## Bullet Resources
* Github - https://github.com/bulletphysics/bullet3
