#import numpy as np
import autograd.numpy as np
from autograd import grad, hessian
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
from numpy.linalg import svd


def np_get_nominal_param():
    nominal_d     = np.array([0.089159,  0.0,    0.0,     0.10915,  0.09465, 0.0823])         # link offset [m]
    nominal_r     = np.array([0.0,      -0.425, -0.39225, 0.0,      0.0,     0.0   ])         # link length [m]  (sometimes denoted by a)
    nominal_alpha = np.array([0.5,       0.0,    0.0,     0.5,     -0.5,     0.0   ]) * np.pi # link twist [rad] 
    nominal_theta_offset = np.array([np.pi,       0.0,    0.0,     0.0,      0.0,     0.0]) # joint angle offset (to be compatible with Rviz)
    nominal_rotparam_we_ee = Rotation.from_euler('zyx', [90, 0, 90], degrees=True).as_rotvec() # rotation vector transformation matrix from wrist_3_link to end-effector link
    print("nominal_d =", nominal_d)
    print("nominal_r =", nominal_r)
    print("nominal_alpha =", nominal_d)
    print("nominal_theta_offset =", nominal_theta_offset)
    print("nominal_rotparam_we_ee =", nominal_rotparam_we_ee)
    ret = np.concatenate([nominal_d, nominal_r, nominal_alpha, nominal_theta_offset, nominal_rotparam_we_ee]).astype(np.float32)
    ret = ret + ret * 0.05*np.random.randn(ret.shape[0])
    return ret 


def np_get_data():
    all_data =  np.loadtxt('endeffector_data.csv', delimiter=',', skiprows=1).astype(np.float32)#[:100]
    position_data   = all_data[:,1:4]
    quaternion_data = all_data[:,4:8]
    theta_data      = all_data[:,9:]
    rotation_data = []
    for i in range(quaternion_data.shape[0]):
        rotation_data.append( Rotation.from_quat(quaternion_data[i]).as_matrix() ) 
    rotation_data = np.array(rotation_data)
    return theta_data, position_data, rotation_data


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
# Rotation vector
def np_p_dash(p, v_bar, th):
    ret  = p * np.cos(th)
    ret += v_bar * ((p*v_bar).sum()) * (1.-np.cos(th))
    ret += np.array([ v_bar[1]*p[2] -v_bar[2]*p[1], v_bar[2]*p[0] -v_bar[0]*p[2], v_bar[0]*p[1] -v_bar[1]*p[0] ]) * np.sin(th)
    return ret

def np_Mat_we_ee( v ):
    th = np.sqrt( (v**2).sum() )
    v_bar = v / th
    u1 = np_p_dash(np.array([1,0,0]), v_bar, th)
    u2 = np_p_dash(np.array([0,1,0]), v_bar, th)
    u3 = np_p_dash(np.array([0,0,1]), v_bar, th)
    totalR = np.vstack([u1,u2,u3]).T
    return np.array([[totalR[0,0],totalR[0,1],totalR[0,2],0],
                     [totalR[1,0],totalR[1,1],totalR[1,2],0],
                     [totalR[2,0],totalR[2,1],totalR[2,2],0],
                     [0,0,0,1]] )


def _np_get_p_R( d, r, alpha, theta_offset, rotparam_we_ee, theta ):

    np_mat_base_i = np_DHMat_im1_i( d[0], 0., 0., theta[0]-theta_offset[0] )
    for i in range(5):
        np_mat_base_i = np_mat_base_i @ np_DHMat_im1_i( d[i+1], r[i], alpha[i], theta[i+1]-theta_offset[i+1] )
    np_mat_base_i = np_mat_base_i @ np_Mat_we_ee(rotparam_we_ee)
    return np_mat_base_i[:3,3], np_mat_base_i[:3,:3]

def np_get_p_R( param, theta ):
    d = param[:6]
    r = param[6:12]
    alpha = param[12:18]
    theta_offset = param[18:24]
    rotparam_we_ee = param[24:]
    return _np_get_p_R( d, r, alpha, theta_offset, rotparam_we_ee, theta )


def np_get_jacobian( eval_param, theta ):

    def fn_p1(param):
        p, R = np_get_p_R( param, theta )
        return p[0]

    def fn_p2(param):
        p, R = np_get_p_R( param, theta )
        return p[1]

    def fn_p3(param):
        p, R = np_get_p_R( param, theta )
        return p[2]

    def fn_R11(param):
        p, R = np_get_p_R( param, theta )
        return R[0,0]

    def fn_R12(param):
        p, R = np_get_p_R( param, theta )
        return R[0,1]

    def fn_R13(param):
        p, R = np_get_p_R( param, theta )
        return R[0,2]

    def fn_R21(param):
        p, R = np_get_p_R( param, theta )
        return R[1,0]

    def fn_R22(param):
        p, R = np_get_p_R( param, theta )
        return R[1,1]

    def fn_R23(param):
        p, R = np_get_p_R( param, theta )
        return R[1,2]

    def fn_R31(param):
        p, R = np_get_p_R( param, theta )
        return R[2,0]

    def fn_R32(param):
        p, R = np_get_p_R( param, theta )
        return R[2,1]

    def fn_R33(param):
        p, R = np_get_p_R( param, theta )
        return R[2,2]

    ret = grad(fn_p1)(eval_param)
    ret = np.vstack([ret, grad(fn_p2)(eval_param)])
    ret = np.vstack([ret, grad(fn_p3)(eval_param)])

    ret = np.vstack([ret, grad(fn_R11)(eval_param)])
    ret = np.vstack([ret, grad(fn_R12)(eval_param)])
    ret = np.vstack([ret, grad(fn_R13)(eval_param)])
    ret = np.vstack([ret, grad(fn_R21)(eval_param)])
    ret = np.vstack([ret, grad(fn_R22)(eval_param)])
    ret = np.vstack([ret, grad(fn_R23)(eval_param)])
    ret = np.vstack([ret, grad(fn_R31)(eval_param)])
    ret = np.vstack([ret, grad(fn_R32)(eval_param)])
    ret = np.vstack([ret, grad(fn_R33)(eval_param)])
    return ret


def np_get_matrix_for_idenfiable_parameter():

    theta_data , _, _ = np_get_data()    
    param0 = np_get_nominal_param()
    total_jac = np_get_jacobian(param0, theta_data[0])
    for i in range(1,theta_data.shape[0]):
        print("Computing Jacobian", 100.*i/theta_data.shape[0], "[%]\r")
        total_jac = np.vstack([total_jac, np_get_jacobian(param0, theta_data[i])])

    u, s, vh = svd(total_jac)

    #np.set_printoptions(precision=6,suppress=True)
    print("s =",s)
    nonzero_num = np.count_nonzero(s>1.e-5)

    #print("nonzero_num =",nonzero_num)
    #print("vh[:nonzero_num].shape =",vh[:nonzero_num].shape)
    return vh[:nonzero_num].T


def train(param_init):

    theta_data, position_data, rotation_data = np_get_data()
    mat = np_get_matrix_for_idenfiable_parameter()

    def total_loss(param_diff):
        param = param_init +(np.dot(mat,param_diff))
        ret=0.
        for i in range(theta_data.shape[0]):
            p, R = np_get_p_R( param, theta_data[i] )
            ret += 100.*np.sum((position_data[i]-p)**2) + np.sum((rotation_data[i]-R)**2)
        return ret

    param_gradient = grad(total_loss)
    param_hessian  = hessian(total_loss)

    param = np.zeros(mat.shape[1])

    res = minimize(total_loss, param, jac=param_gradient, method="L-BFGS-B", options={'disp': True})
    print("res.x",res.x)
    ret = param_init + np.dot(mat,res.x)

    np.set_printoptions(precision=6,suppress=True)
    print("param_init =",param_init)
    print("param_fin =",ret)
    print("param_diff =",ret-param_init)
    return param_init + np.dot(mat,res.x)

param = np_get_nominal_param()
param = train(param)
param = train(param)




