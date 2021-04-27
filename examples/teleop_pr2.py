#!/usr/bin/env python

from __future__ import print_function

import select
import sys
import termios
import tty

from pybullet_tools.pr2_utils import PR2_GROUPS, DRAKE_PR2_URDF
from pybullet_tools.utils import add_data_path, connect, enable_gravity, load_model, \
	joints_from_names, load_pybullet, \
	velocity_control_joints, disconnect, enable_real_time, HideOutput

HELP_MSG = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

MOVE_BINDINGS = {
	'i': (1, 0, 0, 0),
	'o': (1, 0, 0, -1),
	'j': (0, 0, 0, 1),
	'l': (0, 0, 0, -1),
	'u': (1, 0, 0, 1),
	',': (-1, 0, 0, 0),
	'.': (-1, 0, 0, 1),
	'm': (-1, 0, 0, -1),
	'O': (1, -1, 0, 0),
	'I': (1, 0, 0, 0),
	'J': (0, 1, 0, 0),
	'L': (0, -1, 0, 0),
	'U': (1, 1, 0, 0),
	'<': (-1, 0, 0, 0),
	'>': (-1, -1, 0, 0),
	'M': (-1, 1, 0, 0),
	't': (0, 0, 1, 0),
	'b': (0, 0, -1, 0),
}

SPEED_BINDINGS = {
	'q': (1.1, 1.1),
	'z': (.9, .9),
	'w': (1.1, 1),
	'x': (.9, 1),
	'e': (1, 1.1),
	'c': (1, .9),
}

ESCAPE = '\x03'

#####################################

def get_key(settings):
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def print_velocities(speed, turn):
	print("Speed: {} | Turn: {} ".format(speed, turn))

#####################################

def run_simulate(pr2):
	joints = joints_from_names(pr2, PR2_GROUPS['base'])
	dx = dy = dz = dth = 1
	speed, turn = 0.5, 1.0
	while True:
		velocities = [dx * speed, dy * speed, dth * turn]
		velocity_control_joints(pr2, joints, velocities)

def run_thread(pr2):
	joints = joints_from_names(pr2, PR2_GROUPS['base'])
	dx = dy = dz = dth = 0
	speed, turn = 0.5, 1.0
	settings = termios.tcgetattr(sys.stdin)
	try:
		print(HELP_MSG)
		print_velocities(speed, turn)
		while True: # TODO: getKeyboardEvents
			key = get_key(settings)  # Waits until a key is read
			if key in MOVE_BINDINGS:
				dx, dy, dz, dth = MOVE_BINDINGS[key]
			elif key in SPEED_BINDINGS:
				mspeed, mturn = SPEED_BINDINGS[key]
				speed *= mspeed
				turn *= mturn
				print_velocities(speed, turn)
			else:  # When it receives another key
				dx = dy = dz = dth = 0
				if key == ESCAPE:
					break
			# twist.linear.dz = dz * speed
			velocities = [dx * speed, dy * speed, dth * turn]
			velocity_control_joints(pr2, joints, velocities)
	except Exception as e:
		print(e)
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#####################################

def main():
	# https://github.com/ros-teleop/teleop_twist_keyboard
	# http://openrave.org/docs/latest_stable/_modules/openravepy/misc/#SetViewerUserThread

	connect(use_gui=True)
	add_data_path()
	load_pybullet("plane.urdf")
	#load_pybullet("models/table_collision/table.urdf")
	with HideOutput():
		pr2 = load_model(DRAKE_PR2_URDF, fixed_base=True)
	enable_gravity()
	enable_real_time() # TODO: won't work as well on OS X due to simulation thread

	#run_simulate(pr2)
	run_thread(pr2)
	# TODO: keep working on this
	#userthread = threading.Thread(target=run_thread, args=[pr2])
	#userthread.start()
	#userthread.join()

	disconnect()


if __name__=="__main__":
	main()