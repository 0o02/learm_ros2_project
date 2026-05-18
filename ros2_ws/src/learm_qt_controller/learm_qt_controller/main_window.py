import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLineEdit, QLabel,
    QGridLayout, QWidget, QMessageBox, QGroupBox, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self, robot_controller):
        super().__init__()
        self.robot = robot_controller
        self.setWindowTitle("LeArm Controller")
        self.setMinimumSize(400, 300)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # 状态标签
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        main_layout.addWidget(self.status_label)

        # 预设点按钮组
        group_presets = QGroupBox("预设位置")
        preset_layout = QHBoxLayout()
        self.btn_init = QPushButton("回初始点")
        self.btn_home = QPushButton("回工作点")
        preset_layout.addWidget(self.btn_init)
        preset_layout.addWidget(self.btn_home)
        group_presets.setLayout(preset_layout)
        main_layout.addWidget(group_presets)

        # 自定义位姿区域
        group_pose = QGroupBox("自定义末端位置 (base_link 坐标系)")
        pose_layout = QGridLayout()
        pose_layout.addWidget(QLabel("X (米):"), 0, 0)
        self.line_x = QLineEdit("0.2")
        pose_layout.addWidget(self.line_x, 0, 1)

        pose_layout.addWidget(QLabel("Y (米):"), 1, 0)
        self.line_y = QLineEdit("0.0")
        pose_layout.addWidget(self.line_y, 1, 1)

        pose_layout.addWidget(QLabel("Z (米):"), 2, 0)
        self.line_z = QLineEdit("0.2")
        pose_layout.addWidget(self.line_z, 2, 1)

        self.btn_send_pose = QPushButton("移动到指定位置")
        pose_layout.addWidget(self.btn_send_pose, 3, 0, 1, 2)

        # 添加工作空间范围提示
        limits = self.robot.get_workspace_limits()
        self.limit_label = QLabel(
            f"建议范围: X∈[{limits['x'][0]}, {limits['x'][1]}], "
            f"Y∈[{limits['y'][0]}, {limits['y'][1]}], "
            f"Z∈[{limits['z'][0]}, {limits['z'][1]}]"
        )
        self.limit_label.setWordWrap(True)
        pose_layout.addWidget(self.limit_label, 4, 0, 1, 2)

        group_pose.setLayout(pose_layout)
        main_layout.addWidget(group_pose)

        # 急停按钮
        self.btn_stop = QPushButton("急停！")
        self.btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 16px;")
        main_layout.addWidget(self.btn_stop)

        # 连接信号
        self.btn_init.clicked.connect(self.on_init)
        self.btn_home.clicked.connect(self.on_home)
        self.btn_send_pose.clicked.connect(self.on_send_pose)
        self.btn_stop.clicked.connect(self.on_stop)

    def on_init(self):
        self.status_label.setText("正在返回初始点...")
        self.robot.go_init()
        self.status_label.setText("已发送初始点指令")

    def on_home(self):
        self.status_label.setText("正在返回工作点...")
        self.robot.go_home()
        self.status_label.setText("已发送工作点指令")

    def on_send_pose(self):
        try:
            x = float(self.line_x.text())
            y = float(self.line_y.text())
            z = float(self.line_z.text())
        except ValueError:
            QMessageBox.warning(self, "输入错误", "请输入有效的数字")
            return

        # 检查是否在工作空间范围内
        limits = self.robot.get_workspace_limits()
        if not (limits['x'][0] <= x <= limits['x'][1]):
            QMessageBox.warning(self, "超出范围", f"X 超出建议范围 [{limits['x'][0]}, {limits['x'][1]}]")
            return
        if not (limits['y'][0] <= y <= limits['y'][1]):
            QMessageBox.warning(self, "超出范围", f"Y 超出建议范围 [{limits['y'][0]}, {limits['y'][1]}]")
            return
        if not (limits['z'][0] <= z <= limits['z'][1]):
            QMessageBox.warning(self, "超出范围", f"Z 超出建议范围 [{limits['z'][0]}, {limits['z'][1]}]")
            return

        self.status_label.setText(f"正在移动到: X={x:.3f}, Y={y:.3f}, Z={z:.3f} ...")
        self.robot.go_pose(x, y, z, retain_orientation=True)
        self.status_label.setText(f"已发送目标: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    def on_stop(self):
        self.status_label.setText("急停已触发！")
        self.robot.stop()
