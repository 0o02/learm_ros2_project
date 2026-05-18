import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rpy_to_rot(roll, pitch, yaw):
    """绕固定轴 X-Y-Z 的旋转矩阵（对应 URDF rpy）"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                  [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                  [ -sp,          cp*sr,          cp*cr]])
    return R

def dh_transform(origin_xyz, origin_rpy, axis, angle):
    """
    根据 URDF 的 joint 标签构造从 parent 到 child 的变换矩阵。
    origin_xyz: [x,y,z]
    origin_rpy: [r,p,y]
    axis: 关节轴在 child 系中的方向
    angle: 关节转角（弧度）
    """
    x, y, z = origin_xyz
    R_off = rpy_to_rot(*origin_rpy)
    T_off = np.eye(4)
    T_off[:3, :3] = R_off
    T_off[:3, 3] = [x, y, z]

    # 绕 axis 转动 angle 的旋转矩阵（罗德里格斯公式）
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c = np.cos(angle)
    s = np.sin(angle)
    v = 1 - c
    kx, ky, kz = ax
    R_rot = np.array([[kx*kx*v + c,   kx*ky*v - kz*s, kx*kz*v + ky*s],
                      [kx*ky*v + kz*s, ky*ky*v + c,   ky*kz*v - kx*s],
                      [kx*kz*v - ky*s, ky*kz*v + kx*s, kz*kz*v + c  ]])
    T_rot = np.eye(4)
    T_rot[:3, :3] = R_rot

    # URDF 中 origin 的变换是在关节旋转之前应用的：
    # parent -> T_off -> child_initial, 然后 child_initial 绕 (child 系中的 axis) 旋转 angle 得到最终 child。
    # 所以整体为 T = T_off @ T_rot
    return T_off @ T_rot

def draw_coordinate_frame(ax, T, size=0.03, label=None):
    """在 4x4 变换矩阵 T 的位置绘制 RGB 三色坐标轴"""
    origin = T[:3, 3]
    x_axis = T[:3, 0]
    y_axis = T[:3, 1]
    z_axis = T[:3, 2]

    ax.quiver(*origin, *(size*x_axis), color='r', linewidth=2, arrow_length_ratio=0.1)
    ax.quiver(*origin, *(size*y_axis), color='g', linewidth=2, arrow_length_ratio=0.1)
    ax.quiver(*origin, *(size*z_axis), color='b', linewidth=2, arrow_length_ratio=0.1)
    if label:
        ax.text(*(origin + 0.01), label, fontsize=9)

# ---------- 关节参数（直接从 URDF 提取，角度设为 0） ----------
# 每个元素: (parent_link, child_link, origin_xyz, origin_rpy, axis)
joints = [
    ("base_link",    "shoulder_link", (0, 0, 0.059),    (0, 0, np.pi),     (0, 0, 1)),
    ("shoulder_link","humerus_link", (-0.010, 0, 0.028),(0, 0, 0),        (0, 1, 0)),
    ("humerus_link", "forearm_link", (0, 0, 0.105),    (0, 0, 0),        (0, 1, 0)),
    ("forearm_link", "wrist_link",   (0, 0, 0.090),    (0, 0, 0),        (0, 1, 0)),
    ("wrist_link",   "hand_link",    (0, 0, 0.060),    (0, 0, 0),        (0, 0, 1)),
]

# 还可以加上手指的随动关节（角度为 0）
mimic_joints = [
    ("hand_link",      "grip_left_link",  (0, 0.015, 0.030), (0,0,0), (1,0,0)),
    ("hand_link",      "grip_right_link", (0, -0.015, 0.030),(0,0,0), (1,0,0)),
    ("grip_left_link", "finger_left_link",(0, 0, 0.030),     (0,0,0), (1,0,0)),
    ("grip_right_link","finger_right_link",(0, 0, 0.030),    (0,0,0), (1,0,0)),
]

# ---------- 正运动学：计算每个 link 的全局位姿 ----------
links = {"base_link": np.eye(4)}          # base 与世界重合
positions = {"base_link": np.zeros(3)}     # 记录原点，用于画连杆

# 先处理主臂关节（所有角度为 0）
for parent, child, xyz, rpy, axis in joints:
    T_parent = links[parent]
    T_child = T_parent @ dh_transform(xyz, rpy, axis, 0.0)
    links[child] = T_child
    positions[child] = T_child[:3, 3]

# 处理手爪随动关节
for parent, child, xyz, rpy, axis in mimic_joints:
    T_parent = links[parent]
    T_child = T_parent @ dh_transform(xyz, rpy, axis, 0.0)
    links[child] = T_child
    positions[child] = T_child[:3, 3]

# ---------- 绘图 ----------
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 画连杆之间的连线（机械臂主链）
main_chain = ["base_link", "shoulder_link", "humerus_link",
              "forearm_link", "wrist_link", "hand_link"]
for i in range(len(main_chain)-1):
    p1 = positions[main_chain[i]]
    p2 = positions[main_chain[i+1]]
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
            'k--', linewidth=1, alpha=0.5)

# 画手指连杆（可选）
finger_chain = [("hand_link", "grip_left_link"),
                ("hand_link", "grip_right_link"),
                ("grip_left_link", "finger_left_link"),
                ("grip_right_link", "finger_right_link")]
for p, c in finger_chain:
    p1 = positions[p]
    p2 = positions[c]
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
            'grey', linewidth=0.8, alpha=0.4)

# 画所有连杆的坐标系
for name, T in links.items():
    draw_coordinate_frame(ax, T, size=0.03, label=name)

# 视角与标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("LeArm DH Coordinate Frames (all joints at 0°)")
ax.view_init(elev=25, azim=-60)   # 可自行调整视角
ax.set_box_aspect([1,1,1])        # 等比例坐标轴
plt.tight_layout()
plt.show()
