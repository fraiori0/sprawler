import sympy.physics.mechanics as me
from sympy import symbols, simplify, sin, cos
from numpy import pi

# Number of link that compose the body (not counting the girdle)
n = 3

#### DYNAMIC VARIABLES AND DERIVATIVES
# q =   {
# q0-q5 		x,y,z,phi,theta,psi	-->	girdle link position and orientation in the inertial global reference frame
# q6-q9		right leg joint variable
# q10-qN		spine joint variable
#       }
# NOTE: q[0] = q0(t)
q = me.dynamicsymbols('q:' + str(n + 10))
qd = me.dynamicsymbols('qd:' + str(n + 10))
#qd = me.dynamicsymbols('q:' + str(n + 10), level=1)	#defined with same name but different "level" attribute

#### NUMERICAL PARAMETERS AND SYMBOLS

#### REFERENCE FRAMES DEFINITION AND ORIENTATION
inertial_rf = me.ReferenceFrame('SI')

# ang_vel is set using girdle_rf.ang_vel_in(inertial_rf) as a reference
girdle_rf = me.ReferenceFrame('SG')
girdle_rf.orient(inertial_rf, 'Space', (q[3],q[4],q[5]), '123')
girdle_rf.set_ang_vel(inertial_rf, (
	(-sin(q[4])*qd[5] + qd[3])*qd[3]						*girdle_rf.x + 
	(sin(q[3])*cos(q[4])*qd[5] +  cos(q[3])*qd[4])	*girdle_rf.y +
	(-sin(q[3])*qd[4] + cos(q[3])*cos(q[4])*qd[5])	*girdle_rf.z
	))

# XZX rotation from girdle to arm, to keep joint variable as URDF of sprawler (see draw for details on reference frame definition)
girdleR1_rf = me.ReferenceFrame('SGR1')
girdleR1_rf.orient(girdle_rf, 'Axis', (q[6], girdle_rf.x))
girdleR1_rf.set_ang_vel(inertial)

girdleR2_rf = me.ReferenceFrame('SGR2')
girdleR2_rf.orient(girdleR1_rf, 'Axis', ((q[7] - pi/2), girdleR1_rf.z))

armR_rf = me.ReferenceFrame('SAR')
armR_rf.orient(girdleR2_rf, 'Axis', (q[8], girdleR2_rf.x))

forearmR_rf = me.ReferenceFrame('SFR')
forearmR_rf.orient(armR_rf, 'Axis', (q[9],armR_rf.z))

spine1_rf = me.ReferenceFrame('SS1')
spine1_rf.orient(girdle_rf, 'Axis', (q[10], girdle_rf.z))

#### POINTS
G = me.Point('G')
RG = me.Point('RG')
RE = me.Point('RE')
RC = me.Point('RC')



#frames = []
#points = []
#particles =[]