#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtWidgets import QApplication
from learm_qt_controller.robot_controller_node import RobotController
from learm_qt_controller.main_window import MainWindow

def ros_spin(executor):
    executor.spin()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # 创建 ROS 节点
    controller = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    # ROS 在后台线程运行
    ros_thread = threading.Thread(target=ros_spin, args=(executor,), daemon=True)
    ros_thread.start()

    # 创建 QT 界面
    window = MainWindow(controller)
    window.show()

    # 运行 QT 事件循环
    ret = app.exec_()

    # 清理
    executor.cancel()  # 停止executor
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
