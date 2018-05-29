#!/usr/bin/env python

from __future__ import print_function

import select
import sys
import termios
import tty

from pr2_utils import PR2_GROUPS
from utils import add_data_path, connect, enable_gravity, load_model, joints_from_names, load_pybullet, \
	velocity_control_joints, \
	disconnect, enable_real_time

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

def main():
	"""
	https://github.com/ros-teleop/teleop_twist_keyboard
	"""

	connect(use_gui=True)
	add_data_path()
	load_pybullet("plane.urdf")
	#load_pybullet("models/table_collision/table.urdf")
	pr2 = load_model("models/drake/pr2_description/urdf/pr2_simplified.urdf", fixed_base=True)
	joints = joints_from_names(pr2, PR2_GROUPS['base'])
	enable_gravity()
	enable_real_time() # TODO: won't work as well on OS X due to simulation thread

	settings = termios.tcgetattr(sys.stdin)
	speed = 0.5
	turn = 1.0
	dx = dy = dz = dth = 0
	status = 0

	try:
		print(HELP_MSG)
		print_velocities(speed, turn)
		while True:
			key = get_key(settings)  # Waits until a key is read
			if key in MOVE_BINDINGS.keys():
				dx, dy, dz, dth = MOVE_BINDINGS[key]
			elif key in SPEED_BINDINGS.keys():
				mspeed, mturn = SPEED_BINDINGS[key]
				speed *= mspeed
				turn *= mturn
				print_velocities(speed, turn)
				if status == 14:
					print(HELP_MSG)
				status = (status + 1) % 15
			else:  # When it receives another key
				dx = dy = dz = dth = 0
				if key == ESCAPE:
					break
			#twist.linear.dz = dz * speed
			velocities = [dx*speed, dy*speed, dth*turn]
			velocity_control_joints(pr2, joints, velocities)
	except Exception as e:
		print(e)

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

	disconnect()


if __name__=="__main__":
	main()