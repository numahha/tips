import numpy as np
from scipy.spatial.transform import Rotation
import torch
from torch.utils.data import TensorDataset, DataLoader

def torch_DHMat_im1_i( d_i, r_i, alpha_i, theta_i ): # DH matrix from i-1-th to i-th coordinate.
    # input.shape  ... (1), (1), (1), (bathsize)
    # output.shape ... (bathsize,4,4)

    temp_ones  = torch.ones_like(theta_i)
    temp_zeros = torch.zeros_like(theta_i)
    ct = torch.cos(theta_i)
    st = torch.sin(theta_i)
    ca = torch.cos(alpha_i) * temp_ones
    sa = torch.sin(alpha_i) * temp_ones


    row1 = torch.stack((ct, -st, temp_zeros, r_i*temp_ones),dim=1)
    row2 = torch.stack((st*ca, ct*ca, -sa, -d_i*sa),dim=1)
    row3 = torch.stack((st*sa, ct*sa,  ca,  d_i*ca),dim=1)
    row4 = torch.stack((temp_zeros, temp_zeros, temp_zeros, temp_ones),dim=1)
    return  torch.stack((row1,row2,row3,row4),1)

"""
# Euler angle representation
def torch_Mat_we_ee(th):
    # input.shape  ... (3)
    tensor_0 = torch.tensor(0.)#torch.zeros(1)
    tensor_1 = torch.tensor(1.)#torch.ones(1)
    Rot1 = torch.stack([
                torch.stack([torch.cos(th[0]), -torch.sin(th[0]), tensor_0]),
                torch.stack([torch.sin(th[0]), torch.cos(th[0]), tensor_0]),
                torch.stack([tensor_0, tensor_0, tensor_1])]).reshape(3,3)
    Rot2 = torch.stack([
                torch.stack([torch.cos(th[1]), tensor_0, torch.sin(th[1])]),
                torch.stack([tensor_0, tensor_1, tensor_0]),
                torch.stack([-torch.sin(th[1]), tensor_0, torch.cos(th[1])])]).reshape(3,3)
    Rot3 = torch.stack([
                torch.stack([tensor_1, tensor_0, tensor_0]),
                torch.stack([tensor_0, torch.cos(th[1]), -torch.sin(th[1])]),
                torch.stack([tensor_0, torch.sin(th[1]), torch.cos(th[1])])]).reshape(3,3)
    retMat = torch.matmul(Rot3, torch.matmul(Rot2, Rot1))
    retMat2 = torch.hstack([retMat, torch.tensor([[0],[0],[0]])])
    return torch.vstack([retMat2, torch.tensor([[0,0,0,1]])])
"""

# rotation vector representation
def p_dash(p, v_bar, th):
    ret  = p * torch.cos(th) + v_bar * ((p*v_bar).sum()) * (1.-torch.cos(th)) + torch.tensor([ v_bar[1]*p[2] -v_bar[2]*p[1], v_bar[2]*p[0] -v_bar[0]*p[2], v_bar[0]*p[1] -v_bar[1]*p[0] ]) * torch.sin(th)
    return ret

def torch_Mat_we_ee( v ):
    th = torch.sqrt( (v**2).sum() )
    v_bar = v / th
    u1 = p_dash(torch.tensor([1,0,0]), v_bar, th)
    u2 = p_dash(torch.tensor([0,1,0]), v_bar, th)
    u3 = p_dash(torch.tensor([0,0,1]), v_bar, th)
    totalR  = torch.vstack([u1,u2,u3]).T
    totalR2 = torch.hstack([totalR, torch.tensor([[0],[0],[0]])])
    return torch.vstack([totalR2, torch.tensor([[0,0,0,1]])])
#"""



class FK(torch.nn.Module):

    def __init__(self, init_param=None, identifiable_flag=None):
        super(FK, self).__init__()

        if init_param is None:
            init_param = torch.zeros(27)
            init_param[26] = 1.*np.pi

        if identifiable_flag is None:
            identifiable_flag=[True]*25


        """
        identifiable_flag[1] = False
        identifiable_flag[2] = False
        identifiable_flag[6] = False
        identifiable_flag[9] = False
        identifiable_flag[10] = False
        identifiable_flag[11] = False
        identifiable_flag[13] = False
        identifiable_flag[14] = False
        identifiable_flag[19] = False
        identifiable_flag[20] = False
        identifiable_flag[21] = False
        identifiable_flag[22] = False
        identifiable_flag[23] = False
        """

        self.d     = []
        self.r     = []
        self.alpha = []
        self.theta_offset = []
        for i in range(6):
            self.d.append(            torch.nn.Parameter(init_param[i],    requires_grad=identifiable_flag[i]))
            self.r.append(            torch.nn.Parameter(init_param[i+6],  requires_grad=identifiable_flag[i+6]))
            self.alpha.append(        torch.nn.Parameter(init_param[i+12], requires_grad=identifiable_flag[i+12]))
            self.theta_offset.append( torch.nn.Parameter(init_param[i+18], requires_grad=identifiable_flag[i+18]))
        self.d = torch.nn.ParameterList(self.d)
        self.r = torch.nn.ParameterList(self.r)
        self.alpha = torch.nn.ParameterList(self.alpha)
        self.theta_offset = torch.nn.ParameterList(self.theta_offset)

        self.rotparam_we_ee = []
        for i in range(3):
            self.rotparam_we_ee.append( torch.nn.Parameter(init_param[24+i], requires_grad=identifiable_flag[24])) # simultaneously vary/fix 3 parameters
        self.rotparam_we_ee = torch.nn.ParameterList(self.rotparam_we_ee)

        print("self.parameters()",[p for p in self.parameters()])
        self.init_param = torch.stack([p.detach().clone() for p in self.parameters()])


    
    def forward(self, theta):

        self_d = [ p for i,p in enumerate(self.d)]
        self_r = [ p for i,p in enumerate(self.r)]
        self_alpha = [ p for i,p in enumerate(self.alpha)]
        self_theta_offset = [ p for i,p in enumerate(self.theta_offset)]
        self_rotparam_we_ee =  torch.stack([ p for i,p in enumerate(self.rotparam_we_ee)])

        tf_mat1 = torch_DHMat_im1_i( self_d[0], torch.tensor(0.), torch.tensor(0.), theta[:,0]-self_theta_offset[0])        
        tf_mat2 = torch.matmul(tf_mat1, torch_DHMat_im1_i( self_d[1], self_r[0], self_alpha[0], theta[:,1]-self_theta_offset[1]))
        tf_mat3 = torch.matmul(tf_mat2, torch_DHMat_im1_i( self_d[2], self_r[1], self_alpha[1], theta[:,2]-self_theta_offset[2]))
        tf_mat4 = torch.matmul(tf_mat3, torch_DHMat_im1_i( self_d[3], self_r[2], self_alpha[2], theta[:,3]-self_theta_offset[3]))
        tf_mat5 = torch.matmul(tf_mat4, torch_DHMat_im1_i( self_d[4], self_r[3], self_alpha[3], theta[:,4]-self_theta_offset[4]))
        tf_mat6 = torch.matmul(tf_mat5, torch_DHMat_im1_i( self_d[5], self_r[4], self_alpha[4], theta[:,5]-self_theta_offset[5]))
        tf_mat7 = torch.matmul(tf_mat6, torch_Mat_we_ee(self_rotparam_we_ee))
        pred_p = tf_mat7[:,:3,3]
        pred_R = tf_mat7[:,:3,:3]
        return pred_p, pred_R

    def compute_decay(self):
        temp_param = torch.stack([p for p in self.parameters()])
        return 0.00 * torch.sum((temp_param-self.init_param)**2)

    def print_param(self):
        np.set_printoptions(precision=5, suppress=True)
        print("d            = ",np.array([ p.detach().numpy() for i,p in enumerate(model.d)]))
        print("r            = ",np.array([ p.detach().numpy() for i,p in enumerate(model.r)]))
        print("alpha        = ",np.array([ p.detach().numpy() for i,p in enumerate(model.alpha)]))
        print("theta_offset = ",np.array([ p.detach().numpy() for i,p in enumerate(model.theta_offset)]))
        self_rotparam_we_ee =  torch.stack([ p for i,p in enumerate(self.rotparam_we_ee)])
        print("rotparam_we_ee = ",self_rotparam_we_ee.detach().numpy())
        print("RotMat_from_RotVec = ", torch_Mat_we_ee(self_rotparam_we_ee).detach().numpy())


def get_train_loader(batch_size):

    data =  np.loadtxt('endeffector_data.csv', delimiter=',', skiprows=1).astype(np.float32) 

    position   = data[:,1:4]
    quaternion = data[:,4:8]
    theta      = data[:,9:]

    Rot = []
    for i in range(quaternion.shape[0]):
        Rot.append( Rotation.from_quat(quaternion[i]).as_matrix() ) 
    Rot = np.array(Rot)

    theta    = torch.from_numpy(theta)
    position = torch.from_numpy(position)
    Rot      = torch.from_numpy(Rot)

    return DataLoader(TensorDataset(theta, position, Rot ), batch_size=batch_size, shuffle=True,  drop_last=False)


def get_nominal_param():
    nominal_d     = np.array([0.089159,  0.0,    0.0,     0.10915,  0.09465, 0.0823])         # link offset [m]
    nominal_r     = np.array([0.0,      -0.425, -0.39225, 0.0,      0.0,     0.0   ])         # link length [m]  (sometimes denoted by a)
    nominal_alpha = np.array([0.5,       0.0,    0.0,     0.5,     -0.5,     0.0   ]) * np.pi # link twist [rad] 
    nominal_theta_offset = np.array([np.pi,       0.0,    0.0,     0.0,      0.0,     0.0]) # joint angle offset (to be compatible with Rviz)
    nominal_rotparam_we_ee = Rotation.from_euler('zyx', [90, 0, 90], degrees=True).as_rotvec() # rotation vector transformation matrix from wrist_3_link to end-effector link
    #print("nominal_d =", nominal_d)
    #print("nominal_r =", nominal_r)
    #print("nominal_alpha =", nominal_d)
    #print("nominal_theta_offset =", nominal_theta_offset)
    #print("nominal_rotparam_we_ee =", nominal_rotparam_we_ee)
    return torch.from_numpy(np.concatenate([nominal_d, nominal_r, nominal_alpha, nominal_theta_offset, nominal_rotparam_we_ee]).astype(np.float32))



def train(init_param=None, identifiable_flag=None):

    model = FK(init_param=init_param, identifiable_flag=identifiable_flag)
    train_loader = get_train_loader(batch_size=16)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001, weight_decay=0.00)

    for i in range(100):
        if 0==(i%10):
            model.print_param()

        total_loss = 0.
        for th_batch, p_batch, R_batch in train_loader:

            optimizer.zero_grad()
            pred_p, pred_R = model(th_batch)
            loss = 1.*torch.sum((pred_p - p_batch)**2) + 0.01*torch.sum((pred_R - R_batch)**2)
            loss += model.compute_decay()
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        print(i,total_loss)


#import torch.autograd.functional.jacobian as jacobian



"""
def get_jacobian(net, x, noutputs):
    x = x.squeeze()
    n = x.size()[0]
    x = x.repeat(noutputs, 1)
    x.requires_grad_(True)
    p,R = net(x)
    print(p.shape,R.shape)
    y = torch.cat([torch.flatten(p).reshape(3,1), torch.flatten(R).reshape(9,1)])
    y.backward(torch.eye(noutputs))
    return x.grad.data
"""

def jac():

    nominal_param = get_nominal_param()
    model = FK(init_param=nominal_param)
    train_loader = get_train_loader(batch_size=1)

    def flatten_forward(th):
        pred_p, pred_R = model(th)
        #print(pred_p.shape, pred_R.reshape(pred_p.shape[0],9).shape)
        pred = torch.hstack([pred_p, pred_R.reshape(pred_p.shape[0],9)])
        return torch.flatten(pred)
 

    optimizer = torch.optim.Adam(model.parameters(), lr=0.001, weight_decay=0.00)
   
    for th_batch, _, _ in train_loader:
        

        #"""
        #optimizer.zero_grad()
        pred_p, pred_R = model(th_batch)
        temp_flat_pred = torch.stack([torch.flatten(pred_p), torch.flatten(pred_p)])




        loss = 1.*torch.sum((pred_p - p_batch)**2) + 0.01*torch.sum((pred_R - R_batch)**2)
        loss += model.compute_decay()
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
        #"""
        print(torch.autograd.functional.jacobian(flatten_forward,th_batch).reshape)
        aaa
        


jac()
#train()
