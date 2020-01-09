from __future__ import print_function, division
from sympy import symbols, simplify
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point

from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)

inertial_frame = ReferenceFrame('I')
lower_leg_frame = ReferenceFrame('L')
theta1, theta2, theta3 = dynamicsymbols('theta1, theta2, theta3')
lower_leg_frame.orient(inertial_frame, 'Axis', (theta1, inertial_frame.z))

lower_leg_frame.dcm(inertial_frame)
omega1, omega2, omega3 = dynamicsymbols('omega1, omega2, omega3')

lower_leg_frame.set_ang_vel(inertial_frame,omega1*inertial_frame.z)
print(lower_leg_frame.ang_vel_in(inertial_frame))   