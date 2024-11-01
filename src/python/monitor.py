#!/usr/bin/env python3
import sys
from os import getcwd
package_name = "ros2_light"
sys.path.append(getcwd() + f"/src/{package_name}/.venv/lib/python3.10/site-packages")

from PyQt6.QtWidgets import QGraphicsScene, QMainWindow, QApplication
from PyQt6.QtGui import QPen, QColor, QPolygonF, QBrush
from PyQt6.QtCore import QLineF, Qt, QPointF, QObject, QThread, pyqtSignal
from math import cos, sin, degrees, sqrt
import sys, rclpy
from ros2_light_py.UI.GUI import Ui_MainWindow
from ros2_light_py.utilize import Info, normalizeRads
from ros2_light_py.monitor_node import MonitorNode

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(dict)
    stop = False

    def run(self) -> None:
        node =  MonitorNode()
        while not self.stop:
            rclpy.spin_once(node)
            self.progress.emit(node.info)
        node.destroy_node()
        self.finished.emit()

class Monitor(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.MAP_WIDTH = self.ui.map_graphic.geometry().width()
        self.MAP_HEIGHT = self.ui.map_graphic.geometry().height()
        self.MAP_CENTER_X = self.MAP_WIDTH // 2
        self.MAP_CENTER_Y = self.MAP_HEIGHT // 2
        self.mapScene = QGraphicsScene(self.ui.map_graphic)
        self.ui.map_graphic.setScene(self.mapScene)
        self.addCenterLine()
        self.car = None
        self.ui.clear_btn.clicked.connect(self.clearScene)

        self._thread = QThread()
        self._worker = Worker()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._worker.progress.connect(self.update)
        self._thread.start()

    def addCenterLine(self) -> None:
        centerLine = QPen(QColor("#ffffff"), 1, Qt.PenStyle.DashLine)
        self.mapScene.addLine(QLineF(self.MAP_CENTER_X, 0, self.MAP_CENTER_X, self.MAP_HEIGHT), centerLine) 
        self.mapScene.addLine(QLineF(0, self.MAP_CENTER_Y, self.MAP_WIDTH, self.MAP_CENTER_Y), centerLine)
    
    def update(self, value: Info) -> None:
        velocity = value["velocity"]
        heading = normalizeRads(value["euler"].yaw)
        x, y = value["pose"]
        button = value["button"]

        if button[0]:
            self.clearScene()

        self.ui.velocity.display(f"{velocity:.2f}")
        self.ui.heading.display(int(degrees(heading)))

        MAP_SCALE = 25
        sceneX = self.MAP_CENTER_X + x * MAP_SCALE
        sceneY = self.MAP_CENTER_Y - y * MAP_SCALE

        lineSize = 2
        lineColor = QColor("#588ef6")
        self.mapScene.addEllipse(sceneX - lineSize, sceneY - (lineSize / 2), lineSize, lineSize, QPen(lineColor), QBrush(lineColor))
        
        carSize = 0.35 * MAP_SCALE
        halfSize = carSize / 2
        _cos = cos(heading) * halfSize
        _sin = sin(heading) * halfSize
        points = [
            [sceneX + cos(heading) * carSize, 
             sceneY - sin(heading) * carSize],
            [sceneX - sqrt(3) * _cos - _sin,
             sceneY + sqrt(3) * _sin - _cos],
            [sceneX - sqrt(3) * _cos + _sin, 
             sceneY + sqrt(3) * _sin + _cos],
        ]
        carColor = QColor("#3db151")
        poly = QPolygonF([QPointF(point[0], point[1]) for point in points])
        if self.car: self.mapScene.removeItem(self.car)
        self.car = self.mapScene.addPolygon(poly, QPen(carColor), QBrush(carColor))

    def stop(self) -> None:
        self._worker.stop = True

    def clearScene(self) -> None:
        self.mapScene.clear()
        self.addCenterLine()
        self.car = None
        self.ui.map_graphic.setScene(self.mapScene)

if __name__ == "__main__":
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    monitor = Monitor()
    monitor.show()
    app.exec()
    monitor.stop()
    rclpy.shutdown()