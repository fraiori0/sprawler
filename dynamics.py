import sympy.physics.mechanics as me
from sympy import symbols
from numpy import pi

#DYNAMIC VARIABLES AND DERIVATIVES
# q =   {
# q1-q6 		x,y,z,phi,theta,psi	-->	girdle link position and orientation in the inertial global reference frame
# q7-q10		right leg joint variable
# q10-qn		spine joint variable
#       }
q1, q2, q3, q4, q5, q6, q7, q8, q9 ,q10 = me.dynamicsymbols('q1:11')
qd1, qd2, qd3, qd4, qd5, qd6, qd7, qd8, qd9, qd10 = me.dynamicsymbols('q1:11', level=1)

#NUMERICAL PARAMETERS

#REFERENCE FRAMES DEFINITION AND ORIENTATION
inertial_rf = me.ReferenceFrame('SI')
girdle_rf = me.ReferenceFrame('SG')
arm_rf = me.ReferenceFrame('SA')
forearm_rf = me.ReferenceFrame('SF')
spine1_rf = me.ReferenceFrame('SS1')

girdle_rf.orient = (inertial_rf, 'Space', (q4,q5,q6), '123')


#frames = []
#points = []
#particles =[]