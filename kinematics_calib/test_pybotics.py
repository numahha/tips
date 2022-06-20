from pybotics.robot import Robot
from pybotics.predefined_models import ur10

nominal_robot = Robot.from_parameters(ur10())


import pandas as pd

def display_robot_kinematics(robot: Robot):
    df = pd.DataFrame(robot.kinematic_chain.matrix)
    df.columns = ["alpha", "a", "theta", "d"]
    display(df)

display_robot_kinematics(nominal_robot)




import numpy as np
from copy import deepcopy

real_robot = deepcopy(nominal_robot)

# let's pretend our real robot has small joint offsets
# in real life, this would be a joint mastering issue (level-1 calibration)
# https://en.wikipedia.org/wiki/Robot_calibration
for link in real_robot.kinematic_chain.links:
    link.theta += np.random.uniform(
        low=np.deg2rad(-0.1),
        high=np.deg2rad(0.1)
    )

display_robot_kinematics(real_robot)


joints = []
positions = []
for i in range(1000):
    q = real_robot.random_joints()
    pose = real_robot.fk(q)
    
    joints.append(q)
    positions.append(pose[:-1,-1])



pd.DataFrame(joints).describe()


pd.DataFrame(positions, columns=['x','y','z']).describe()


from sklearn.model_selection import train_test_split
split = train_test_split(joints, positions, test_size=0.3)

train_joints = split[0]
test_joints = split[1]

train_positions = split[2]
test_positions = split[3]


from pybotics.optimization import compute_absolute_errors

nominal_errors = compute_absolute_errors(
    qs=test_joints,
    positions=test_positions,
    robot=nominal_robot
)

display(pd.Series(nominal_errors).describe())


from pybotics.optimization import OptimizationHandler

# init calibration handler
handler = OptimizationHandler(nominal_robot)

# set handler to solve for theta parameters
kc_mask_matrix = np.zeros_like(nominal_robot.kinematic_chain.matrix, dtype=bool)
kc_mask_matrix[:,2] = True
display(kc_mask_matrix)

handler.kinematic_chain_mask = kc_mask_matrix.ravel()


from scipy.optimize import least_squares
from pybotics.optimization import optimize_accuracy

# run optimization
result = least_squares(
    fun=optimize_accuracy,
    x0=handler.generate_optimization_vector(),
    args=(handler, train_joints, train_positions),
    verbose=2
)  # type: scipy.optimize.OptimizeResult


calibrated_robot = handler.robot
calibrated_errors = compute_absolute_errors(
    qs=test_joints,
    positions=test_positions,
    robot=calibrated_robot
)

display(pd.Series(calibrated_errors).describe())


import matplotlib.pyplot as plt

display_robot_kinematics(calibrated_robot)

#%matplotlib inline

plt.xscale("log")
plt.hist(nominal_errors, color="C0", label="Nominal")
plt.hist(calibrated_errors, color="C1", label="Calibrated")

plt.legend()
plt.xlabel("Absolute Error [mm]")
plt.ylabel("Frequency")
plt.show()



