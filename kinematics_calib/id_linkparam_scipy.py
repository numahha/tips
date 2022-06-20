#import numpy as np
import autograd.numpy as np
from autograd import grad, hessian
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


def np_DHMat_im1_i( d_i, r_i, alpha_i, theta_i ): # DH matrix from i-1-th to i-th coordinate.
    ct = np.cos(theta_i)
    st = np.sin(theta_i)
    ca = np.cos(alpha_i)
    sa = np.sin(alpha_i)
    return np.array([[   ct,   -st,  0.,     r_i],
                     [ca*st, ca*ct, -sa, -d_i*sa],
                     [sa*st, sa*ct,  ca,  d_i*ca],
                     [   0.,    0.,  0.,      1.]])

"""
# Euler angle
def np_Mat_we_ee(th):
    Rot1 = np.array([[np.cos(th[0]), -np.sin(th[0]), 0],
                    [np.sin(th[0]),  np.cos(th[0]), 0],
                    [0,  0          , 1]])
    Rot2 = np.array([[np.cos(th[1]), 0, np.sin(th[1])],
                    [0,  1          , 0],
                    [-np.sin(th[1]), 0, np.cos(th[1])]])
    Rot3 = np.array([[1, 0,  0],
                     [0, np.cos(th[2]), -np.sin(th[2])],
                     [0, np.sin(th[2]),  np.cos(th[2])]])
    totalR = Rot3 @ Rot2 @ Rot1
    return np.array([[totalR[0,0],totalR[0,1],totalR[0,2],0],
                     [totalR[1,0],totalR[1,1],totalR[1,2],0],
                     [totalR[2,0],totalR[2,1],totalR[2,2],0],
                     [0,0,0,1]] )
"""
def p_dash(p, v_bar, th):
    ret  = p * np.cos(th)
    ret += v_bar * ((p*v_bar).sum()) * (1.-np.cos(th))
    ret += np.array([ v_bar[1]*p[2] -v_bar[2]*p[1], v_bar[2]*p[0] -v_bar[0]*p[2], v_bar[0]*p[1] -v_bar[1]*p[0] ]) * np.sin(th)
    return ret

def np_Mat_we_ee( v ):
    th = np.sqrt( (v**2).sum() )
    v_bar = v / th
    u1 = p_dash(np.array([1,0,0]), v_bar, th)
    u2 = p_dash(np.array([0,1,0]), v_bar, th)
    u3 = p_dash(np.array([0,0,1]), v_bar, th)
    totalR = np.vstack([u1,u2,u3]).T
    return np.array([[totalR[0,0],totalR[0,1],totalR[0,2],0],
                     [totalR[1,0],totalR[1,1],totalR[1,2],0],
                     [totalR[2,0],totalR[2,1],totalR[2,2],0],
                     [0,0,0,1]] )


def get_p_R( d, r, alpha, theta_offset, rotparam_we_ee, theta ):

    np_mat_base_i = np_DHMat_im1_i( d[0], 0., 0., theta[0]-theta_offset[0] )
    for i in range(5):
        np_mat_base_i = np_mat_base_i @ np_DHMat_im1_i( d[i+1], r[i], alpha[i], theta[i+1]-theta_offset[i+1] )
    np_mat_base_i = np_mat_base_i @ np_Mat_we_ee(rotparam_we_ee)
    return np_mat_base_i[:3,3], np_mat_base_i[:3,:3]


def loss_function( param, theta, position, rotation ):
    d = param[:6]
    r = param[6:12]
    alpha = param[12:18]
    theta_offset = param[18:24]
    rotparam_we_ee = param[24:]
    p, R = get_p_R( d, r, alpha, theta_offset, rotparam_we_ee, theta )
    return 100.*np.sum((position-p)**2) + np.sum((rotation-R)**2)


all_data =  np.loadtxt('endeffector_data.csv', delimiter=',', skiprows=1).astype(np.float32) 
position_data   = all_data[:,1:4]
quaternion_data = all_data[:,4:8]
theta_data      = all_data[:,9:]
rotation_data = []
for i in range(quaternion_data.shape[0]):
    rotation_data.append( Rotation.from_quat(quaternion_data[i]).as_matrix() ) 
rotation_data = np.array(rotation_data)



def total_loss(param):
    ret=0.
    for i in range(all_data.shape[0]):
        ret += loss_function( param, theta_data[i], position_data[i], rotation_data[i] )
    return ret

param_gradient = grad(total_loss)
param_hessian  = hessian(total_loss)

param0 = np.zeros(27)
param0[-1] = 1.



rotvec = np.array([0, np.pi/2, np.pi/3])
rot = Rotation.from_rotvec(rotvec)
print("rot.as_matrix",rot.as_matrix())
print("rot",np_Mat_we_ee( rotvec ))


#"""
res = minimize(total_loss, param0, jac=param_gradient, method="L-BFGS-B", options={'disp': True})

print(res)


print("d = ",res.x[:6])
print("r = ",res.x[6:12])
print("alpha = ",res.x[12:18])
print("theta_offset = ",res.x[18:24])
print("rotparam_we_ee = ",res.x[24:])
#"""
"""
param0 = res.x
res = minimize(total_loss, param0, jac=param_gradient, hess=param_hessian, method="trust-ncg", options={'disp': True})
print("d = ",res.x[:6])
print("r = ",res.x[6:12])
print("alpha = ",res.x[12:18])
print("theta_offset = ",res.x[18:24])
print("euler_we_ee = ",res.x[24:])
"""



"""
# DH parameters from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
true_d     = np.array([0.089159,  0.0,    0.0,     0.10915,  0.09465, 0.0823])         # link offset [m]
true_r     = np.array([0.0,      -0.425, -0.39225, 0.0,      0.0,     0.0   ])         # link length [m]  (sometimes denoted by a)
true_alpha = np.array([0.5,       0.0,    0.0,     0.5,     -0.5,     0.0   ]) * np.pi # link twist [rad] 


# joint angle offset (to be compatible with Rviz)
true_theta_offset = np.array([np.pi,       0.0,    0.0,     0.0,      0.0,     0.0])

true_euler_we_ee = np.array([0.5*np.pi, 0., 0.5*np.pi])


theta= np.array([0,       0.0,    0.0,     0.0,      0.0,     0.0])
"""

