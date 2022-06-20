import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# rosnode

theta = np.array([0.0, 0., 0.0, 0.0, 0.0, 0.0]) # joint angle [rad]


def np_DHMat_im1_i( d_i, r_i, alpha_i, theta_i ): # DH matrix from i-1-th to i-th coordinate.
    ct = np.cos(theta_i)
    st = np.sin(theta_i)
    ca = np.cos(alpha_i)
    sa = np.sin(alpha_i)
    return np.array([[   ct,   -st,  0.,     r_i],
                     [ca*st, ca*ct, -sa, -d_i*sa],
                     [sa*st, sa*ct,  ca,  d_i*ca],
                     [   0.,    0.,  0.,      1.]])

def np_Mat_we_ee(th1, th2, th3):
    retMat = np.eye(4)
    Rot1 = np.array([[np.cos(th1), -np.sin(th1), 0],
                    [np.sin(th1),  np.cos(th1), 0],
                    [0,  0          , 1]])
    Rot2 = np.array([[np.cos(th2), 0, np.sin(th2)],
                    [0,  1          , 0],
                    [-np.sin(th2), 0, np.cos(th2)]])
    Rot3 = np.array([[1, 0,  0],
                     [0, np.cos(th3), -np.sin(th3)],
                     [0, np.sin(th3),  np.cos(th3)]])
    retMat[:3,:3] = Rot3 @ Rot2 @ Rot1
    return retMat



# DH parameters from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
true_d     = np.array([0.089159,  0.0,    0.0,     0.10915,  0.09465, 0.0823])         # link offset [m]
true_r     = np.array([0.0,      -0.425, -0.39225, 0.0,      0.0,     0.0   ])         # link length [m]  (sometimes denoted by a)
true_alpha = np.array([0.5,       0.0,    0.0,     0.5,     -0.5,     0.0   ]) * np.pi # link twist [rad] 


# Transformation matrix from wrist_3_link to end-effector link
Mat_we_ee = np.eye(4)
Mat_we_ee[:3,:3] = Rotation.from_euler('zyx', [90, 0, 90], degrees=True).as_matrix()


# joint angle offset (to be compatible with Rviz)
true_theta_offset = np.array([np.pi,       0.0,    0.0,     0.0,      0.0,     0.0])




# preprocessing
true_d     = np.append(0., true_d)
true_r     = np.append(0., true_r)
true_alpha = np.append(0., true_alpha)
temp_theta = np.append(0., theta - true_theta_offset)

axis_len = 0.02
np_mat_base_i = np.eye(4)
origin_list = (np_mat_base_i @ np.array([0,0,0,1])       )[np.newaxis]
x_list      = (np_mat_base_i @ np.array([axis_len,0,0,1]))[np.newaxis]
y_list      = (np_mat_base_i @ np.array([0,axis_len,0,1]))[np.newaxis]
z_list      = (np_mat_base_i @ np.array([0,0,axis_len,1]))[np.newaxis]
for i in range(6):
    np_mat_base_i = np_mat_base_i @ np_DHMat_im1_i( true_d[i+1], true_r[i], true_alpha[i], temp_theta[i+1] )
    origin_list = np.concatenate([  origin_list, (np_mat_base_i @ np.array([0,0,0,1])       )[np.newaxis]  ])
    x_list      = np.concatenate([  x_list,      (np_mat_base_i @ np.array([axis_len,0,0,1]))[np.newaxis]  ])
    y_list      = np.concatenate([  y_list,      (np_mat_base_i @ np.array([0,axis_len,0,1]))[np.newaxis]  ])
    z_list      = np.concatenate([  z_list,      (np_mat_base_i @ np.array([0,0,axis_len,1]))[np.newaxis]  ])
np_mat_base_i = np_mat_base_i @ Mat_we_ee
origin_list = np.concatenate([  origin_list, (np_mat_base_i @ np.array([0,0,0,1])       )[np.newaxis]  ])
x_list      = np.concatenate([  x_list,      (np_mat_base_i @ np.array([2*axis_len,0,0,1]))[np.newaxis]  ])
y_list      = np.concatenate([  y_list,      (np_mat_base_i @ np.array([0,2*axis_len,0,1]))[np.newaxis]  ])
z_list      = np.concatenate([  z_list,      (np_mat_base_i @ np.array([0,0,2*axis_len,1]))[np.newaxis]  ])

print("Mat_we_ee =",Mat_we_ee)
print("np_mat_base_i = ",np_mat_base_i)
print("r = ",np_mat_base_i[:3,3])
print("R = ",np_mat_base_i[:3,:3])
print("q = ",Rotation.from_matrix(np_mat_base_i[:3,:3]).as_quat())



fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
for i in range(6+1):
    ax.plot([origin_list[i,0],x_list[i,0]], [origin_list[i,1],x_list[i,1]], [origin_list[i,2],x_list[i,2]], color='r')
    ax.plot([origin_list[i,0],y_list[i,0]], [origin_list[i,1],y_list[i,1]], [origin_list[i,2],y_list[i,2]], color='g')
    ax.plot([origin_list[i,0],z_list[i,0]], [origin_list[i,1],z_list[i,1]], [origin_list[i,2],z_list[i,2]], color='b')
i=7
ax.plot([origin_list[i,0],x_list[i,0]], [origin_list[i,1],x_list[i,1]], [origin_list[i,2],x_list[i,2]], color='r', linewidth=3.)
ax.plot([origin_list[i,0],y_list[i,0]], [origin_list[i,1],y_list[i,1]], [origin_list[i,2],y_list[i,2]], color='g', linewidth=3.)
ax.plot([origin_list[i,0],z_list[i,0]], [origin_list[i,1],z_list[i,1]], [origin_list[i,2],z_list[i,2]], color='b', linewidth=3.)


# keep xyz-scales being the same
Xmin = min([origin_list[:,0].min(), x_list[:,0].min(), y_list[:,0].min(), z_list[:,0].min()])
Xmax = max([origin_list[:,0].max(), x_list[:,0].max(), y_list[:,0].max(), z_list[:,0].max()])
Ymin = min([origin_list[:,1].min(), x_list[:,1].min(), y_list[:,1].min(), z_list[:,1].min()])
Ymax = max([origin_list[:,1].max(), x_list[:,1].max(), y_list[:,1].max(), z_list[:,1].max()])
Zmin = min([origin_list[:,2].min(), x_list[:,2].min(), y_list[:,2].min(), z_list[:,2].min()])
Zmax = max([origin_list[:,2].max(), x_list[:,2].max(), y_list[:,2].max(), z_list[:,2].max()])
max_range = np.array([Xmax-Xmin, Ymax-Ymin, Zmax-Zmin]).max() * 0.5
mid_x = (Xmax+Xmin) * 0.5
mid_y = (Ymax+Ymin) * 0.5
mid_z = (Zmax+Zmin) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()


