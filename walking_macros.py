#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script that holds useful macros for DCM walking tuning
'''

import numpy as np
from collections import defaultdict

# HIGH-LEVEL
# Walking
# foot swing trajectory
Ts     = 0.24   # desired stance phase duration [s]
Ts_min = 0.20   # minimum stance duration       [s]
Ts_max = 0.28   # maximum stance duration       [s]
T_buff = 0.05   # stop plan before T - T_buff   [s]

Txi = 0.00      # x stay before Txi
Txn = 0.05      # x go to nominal before Txn
Txf = 0.00      # x arrive before T - Txf

Tyi = 0.00      # y stay before Tyi
Tyn = 0.05      # y go to nominal before Tyn
Tyf = 0.00      # y arrive before T - Tyf

Tzm = 0.10      # desired swing apex time [s]
Tzf = 0.00      # z arrive before T - Tzf

zm_l = 0.035    # left  swing apex height [m]
zm_r = 0.035    # right swing apex height [m]

zf_l = -0.002   # left  swing final height [m]
zf_r = -0.002   # right swing final height [m]

hz = 0.365      # desired CoM height [m]

yaw_f_offset = 0.02  # foot yaw offset [rad]

# kinematic reachability [m]
lx  = 0.20      # max longitudinal step length
lyi = 0.04      # min lateral distance between feet
lyo = 0.20      # max lateral distance between feet

# velocity offset compensation [m]
bx_offset = +0.010  # set to negative if BRUCE tends to go forward
by_offset = +0.000  # set to negative if BRUCE tends to go left

# Stance
ka = -0.0       # x position of CoM from the center of foot, in scale of 1/2 foot length
                # ka = 1 puts CoM at the front tip of foot

# TOP-LEVEL
COM_POSITION_X     = 0
COM_POSITION_Y     = 1
COM_POSITION_Z     = 2

BODY_ORIENTATION_X = 3
BODY_ORIENTATION_Y = 4
BODY_ORIENTATION_Z = 5

COM_VELOCITY_X     = 6
COM_VELOCITY_Y     = 7
BODY_YAW_RATE      = 8

FOOT_YAW_RIGHT     = 9
FOOT_YAW_LEFT      = 10
FOOT_CLEARANCE     = 11

COOLING_SPEED      = 12

PARAMETER_ID_LIST      = range(13)
PARAMETER_INCREMENT    = [ 0.05,  0.05,  0.002,       1,     1,     2,    0.01,  0.01,     1,       1,     1,  0.01,       1]
PARAMETER_DEFAULT      = [ 0.00,  0.00,  0.000,       0,     0,     0,     0.0,   0.0,     0,       0,     0,  0.05,       0]
PARAMETER_MAX          = [ 0.20,  0.50,  0.020,       8,    10,    20,    0.10,  0.10,    15,      10,    10,  0.08,       5]
PARAMETER_MIN          = [-0.20, -0.50, -0.160,      -8,   -10,   -20,   -0.10, -0.10,   -15,     -10,   -10,  0.03,       0]
PARAMETER_BUTTON_PLUS  = [  'g',   'j',    'l',     'y',   'i',   'p',     'w',   'a',   'q',     'x',   'v',   'm',     '=']
PARAMETER_BUTTON_MINUS = [  'f',   'h',    'k',     't',   'u',   'o',     's',   'd',   'e',     'z',   'c',   'n',     '-']
PARAMETER_TYPE         = ['len', 'len',  'len',   'ang', 'ang', 'ang',   'len', 'len', 'ang',   'ang', 'ang', 'len',   'len']
PARAMETER_RECOVER      = [  'y',   'y',    'y',     'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',   'y',     'n']

BALANCE = 0
WALK    = 1
PARAMETER_MODE_LIST = {COM_POSITION_X:     [BALANCE],
                       COM_POSITION_Y:     [BALANCE],
                       COM_POSITION_Z:     [BALANCE],
                       BODY_ORIENTATION_X: [BALANCE],
                       BODY_ORIENTATION_Y: [BALANCE],
                       BODY_ORIENTATION_Z: [BALANCE],
                       COM_VELOCITY_X:     [WALK],
                       COM_VELOCITY_Y:     [WALK],
                       BODY_YAW_RATE:      [WALK],
                       FOOT_YAW_RIGHT:     [WALK],
                       FOOT_YAW_LEFT:      [WALK],
                       FOOT_CLEARANCE:     [WALK],
                       COOLING_SPEED:      [BALANCE, WALK]
                       }

# wave trajectory
arm_position_nominal = np.array([-0.7,  1.3,  2.0, 
                                  0.7, -1.3, -2.0])
arm_position_goal    = np.array([0.0, -1.2, 0.0,
                                 0.0,  1.2, 0.0])
arm_trajectory = defaultdict()

for i in range(6):
    arm_trajectory[i] = np.linspace(arm_position_nominal[i], arm_position_goal[i], 20, endpoint=True)

traj_time = np.linspace(0, 2.75 * 2 * np.pi, 30)
for tdx in traj_time:
    arm_trajectory[1] = np.append(arm_trajectory[1], arm_position_goal[1] - 0.3 * np.sin(tdx))
    arm_trajectory[4] = np.append(arm_trajectory[4], arm_position_goal[4] + 0.3 * np.sin(tdx))

    for i in [0, 2, 3, 5]:
        arm_trajectory[i] = np.append(arm_trajectory[i], arm_position_goal[i])

for i in range(6):
    arm_trajectory[i] = np.append(arm_trajectory[i], np.linspace(arm_trajectory[i][-1], arm_position_nominal[i], 20, endpoint=True))


def get_arm_joint_targets(t_global,
                          Ts_step,
                          vxd,
                          vyd,
                          arm_nominal=arm_position_nominal,
                          base_amp=0.15,
                          k_speed=1.0,
                          max_amp=2):

    # 步态周期（左右腿各一步）
    gait_period = max(2.0 * Ts_step, 1e-3)
    omega = 2.0 * np.pi / gait_period  # 角频率

    # 当前“行走速度标量”，决定摆幅
    v_mag = np.sqrt(vxd ** 2 + vyd ** 2)
    amp = base_amp + k_speed * v_mag
    amp = np.clip(amp, 0.0, max_amp)

    # 基于全局时间的相位
    phase = omega * t_global

    # 以名义姿态为基准
    q_arm = np.array(arm_nominal, dtype=float)

    # 假设关节 0 / 3 主要是肩前后摆（你可以根据实际关节映射修改）
    # 右臂正摆、左臂反摆
    q_arm[0] += amp * np.sin(phase)      # 右肩：前后摆
    q_arm[3] += amp * np.sin(phase)      # 左肩：反向

    # 如果想在肘部也加一点小摆动，可以解注释下面两行：
    # q_arm[2] += 0.3 * amp * np.sin(phase)
    # q_arm[5] -= 0.3 * amp * np.sin(phase)

    return q_arm
