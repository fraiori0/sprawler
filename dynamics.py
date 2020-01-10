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
# LENGTH : girdle connection, arm, forearm, spine link
l_g, l_a, l_f, l_s = symbols('l_g, l_a, l_f, l_s')

#### REFERENCE FRAMES DEFINITION AND ORIENTATION
frames=[]

inertial_rf = me.ReferenceFrame('SI')
frames.append(inertial_rf)

# ang_vel is set using girdle_rf.ang_vel_in(inertial_rf) as a reference
girdle_rf = me.ReferenceFrame('SG')
girdle_rf.orient(inertial_rf, 'Space', (q[3],q[4],q[5]), '123')
girdle_rf.set_ang_vel(inertial_rf, (
	(-sin(q[4])*qd[5] + qd[3])*qd[3]						*girdle_rf.x + 
	(sin(q[3])*cos(q[4])*qd[5] +  cos(q[3])*qd[4])	*girdle_rf.y +
	(-sin(q[3])*qd[4] + cos(q[3])*cos(q[4])*qd[5])	*girdle_rf.z
	))
frames.append(girdle_rf)

# XZX rotation from girdle to arm, to keep joint variable as URDF of sprawler (see draw for details on reference frame definition)
# R1 and R2 are right arm's auxiliary reference frame, used to make it easier to compose angular velocity
girdleR1_rf = me.ReferenceFrame('SGR1')
girdleR1_rf.orient(girdle_rf, 'Axis', (q[6], girdle_rf.x))
girdleR1_rf.set_ang_vel(girdle_rf, qd[6]*girdle_rf.x)
frames.append(girdleR1_rf)

girdleR2_rf = me.ReferenceFrame('SGR2')
girdleR2_rf.orient(girdleR1_rf, 'Axis', ((q[7] - pi/2), girdleR1_rf.z))
girdleR2_rf.set_ang_vel(girdleR1_rf, qd[7]*girdleR1_rf.z)
frames.append(girdleR2_rf)

armR_rf = me.ReferenceFrame('SAR')
armR_rf.orient(girdleR2_rf, 'Axis', (q[8], girdleR2_rf.x))
armR_rf.set_ang_vel(girdleR2_rf, qd[8]*girdleR2_rf.x)
frames.append(armR_rf)

forearmR_rf = me.ReferenceFrame('SFR')
forearmR_rf.orient(armR_rf, 'Axis', (q[9],armR_rf.z))
forearmR_rf.set_ang_vel(armR_rf, qd[9]*armR_rf.z)
frames.append(forearmR_rf)

spine1_rf = me.ReferenceFrame('SS1')
spine1_rf.orient(girdle_rf, 'Axis', (q[10], girdle_rf.z))
spine1_rf.set_ang_vel(girdle_rf, qd[10]*girdle_rf.z)
frames.append(spine1_rf)

#### POINTS
points = []

O = me.Point('O')	#origin of the inertial reference frame
O.set_vel(inertial_rf, 0)
points.append(O)

G = me.Point('G')	#girdle center
G.set_pos(O, q[0]*inertial_rf.x + q[1]*inertial_rf.y + q[2]*inertial_rf.z)
G.set_vel(inertial_rf, qd[0]*inertial_rf.x + qd[1]*inertial_rf.y + qd[2]*inertial_rf.z)
points.append(G)

RG = me.Point('RG')
RG.set_pos(G, -l_g*girdle_rf.y)
RG.set_vel(girdle_rf, 0)
RG.v2pt_theory(G, inertial_rf, girdle_rf)
points.append(RG)

RE = me.Point('RE')
RE.set_pos(RG, l_a*armR_rf.x)
RE.v2pt_theory(RG, inertial_rf, armR_rf)
points.append(RE)

RC = me.Point('RC')
RC.set_pos(RE, -l_a*girdleR2_rf.x)
RC.v2pt_theory(RE, inertial_rf, girdleR2_rf)
points.append(RC)

SG = me.Point.locatenew(G,'SG', -l_s*girdle_rf.x)
SG.v2pt_theory(G, inertial_rf, girdle_rf)
points.append(SG)

S1 = me.Point('S1')	
S1.set_pos(SG, -l_s*spine1_rf.x)
S1.v2pt_theory(SG, inertial_rf, spine1_rf)
points.append(S1)
#print(S1.vel(inertial_rf))

#### PARTICLES (Point masses are used instead of rigid body to simplify the model)
#MASS : spine link, foot
m_spine, m_contact = symbols('m_spine, m_contact')

particles = []

g_p = me.Particle('g_p', G, m_spine)
particles.append(g_p)
rc_p = me.Particle('rc_p', RC, m_contact)
particles.append(rc_p)
sg_p = me.Particle('sg_p', SG, m_spine)
particles.append(sg_p)
s1_p = me.Particle('s1_p', S1, m_spine)
particles.append(s1_p)

#### FORCES
# force = (point, vector)
# torque = (reference frame, vector)

#GRAVITY
g = symbols('g')
def apply_gravity_force (particle, inertial_reference_frame):
	#this function assumes that a variable called 'g' exists and corresponds to a symbol representing gravity constant
	force = (particle.point, -particle.mass*g*inertial_reference_frame.z)
	return force

gravity_forces = []
for part in particles:
	gravity_forces.append(apply_gravity_force(part, inertial_rf))
#print(gravity_forces)

#JOINT'S TORQUES
# Internal torques need to be defined respecting Newton's third law,
# so as one torque acting on the upper link and an opposite torque acting on the lower link
joint_torques=[]
t_RG0, t_RG1, t_RG2, t_RE, t_SG = symbols('t_RG0, t_RG1, t_RG2, t_RE, t_SG')

joint_torques.append((girdle_rf, -t_RG0*girdle_rf.x))
joint_torques.append((girdleR1_rf, t_RG0*girdleR1_rf.z))

joint_torques.append((girdleR1_rf, -t_RG1*girdleR1_rf.z))
joint_torques.append((girdleR2_rf, t_RG1*girdleR2_rf.z))

joint_torques.append((girdleR2_rf, -t_RG2*girdleR2_rf.x))
joint_torques.append((armR_rf, t_RG2*armR_rf.x))

joint_torques.append((armR_rf, -t_RE*armR_rf.y))
joint_torques.append((forearmR_rf, t_RE*forearmR_rf.z))

joint_torques.append((girdle_rf, -t_SG*girdle_rf.z))
joint_torques.append((spine1_rf, t_SG*spine1_rf.z))

# ALL LOADS
loads = [*gravity_forces, *joint_torques]