from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt5 import QtGui
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl
import sys
import numpy as np

# External files needed:
from qtframeV3 import QtMain  # PyQt framework
from DOF3_Robot import Robot # OpenGL Robot object

"""
Simulation of a 6 DOF robot.
- Cédric Wassenaar & Joeri Verkerk
- C.M.Wassenaar@student.hhs.nl
- 24-09-2018

DEPENDENCIES:
- DOF3_Robot.py
- qtframeV3.py
- rm_utilitiesV3.py
- PyQt5

For:
- kinematics see rm_utilitiesV3 & DOF3_Robot
- PyQt5 implementation see qtframeV3 & main
- 3D rendering see class RenderWindow
"""


class RenderWindow(QWidget):
    """Class to render a 3D view, inherits QWidget"""
    def __init__(self, parent_class):
        QWidget.__init__(self)
        self.vbox = QVBoxLayout(self)
        self.parent_class = parent_class
        self.view3D = gl.GLViewWidget()
        self.fps = 60
        self.view3D.opts['elevation'] = 45
        self.view3D.opts['azimuth'] = -45
        self.view3D.opts['fov'] = 2
        self.view3D.setCameraPosition(distance=500)
        self.grid0 = gl.GLGridItem()
        self.grid0.setSize(8,8,1)
        self.view3D.addItem(self.grid0)
        self.vbox.addWidget(self.view3D)
        self.setLayout(self.vbox)
        self.timer = QTimer()

        # Robot declaration using the DOF3_Robot
        self.r1 = Robot(self.view3D)

        # Sets the robot trajectory as: array(X, Y, Z, Ax, Ay, Az) (in radians)
        self.r1.set_new_trajectory(np.array([[1.5, 0, 1, 0, -1.57079, 0],
                                   [1.5, 0, 2.5, 0, -1.57079, 0],
                                   [1.5, 0, 1, 0, -1.57079, 1.57079],
                                   [1.5, 1, 1, 0, -1.57079, 0],
                                   [1.5, 0, 1, 1.57079, 0, 1.57079],
                                   [1.5, 0, 1, 0, -1.57079, 0],
                                   ]), 30)
        self.r1.link[0].frame.translate(0, -2, 0)
        self.r2 = Robot(self.view3D)
        self.r2.set_new_trajectory(np.array([[1.5, 0, 1, 0, -1.57079, 0],
                                   [1.5, 0, 2.5, 0, -1.57079, 0],
                                   [1.5, 0, 1, 0, -1.57079, 0],
                                   [1.5, 1, 1, 0, -1.57079, 0],
                                   [1.5, 1, 2.5, 0, -1.57079, 0],
                                   [1.5, 0, 2.5, 0, -1.57079, 0],
                                   [1.5, 0, 1, 0, -1.57079, 0],
                                   ]), 30)
        self.r3 = Robot(self.view3D)
        self.r3.set_new_trajectory(np.array([[1.5, 0, 1, 0, -1.57079, 0],
                                   [1.5, 0, 2.5, 0, -1.57079, 0],
                                   [1.5, 0, 1, 1.57079, -1.57079, 0],
                                   [1.5, 1, 1, 0, -1.57079, 0],
                                   [1.5, 0, 1, 0, -1.57079, 0],
                                   ]), 30)
        self.r3.link[0].frame.translate(0, 2, 0)
        # Start frame update timer
        self.start()

        # Create linker to function when system closes
        self.destroyed.connect(self._on_destroyed)

    def update_window(self):
        """Render new frame of 3D view"""
        # Update robot position
        self.r1.update_window()
        self.r2.update_window()
        self.r3.update_window()

        # now update the OpenGL graphics in window
        self.view3D.updateGL()

    def start(self):
        """"Starts timer to call window update"""
        self.timer.timeout.connect(self.update_window)
        self.timer.start(int(1000/self.fps))

    def update_timer(self, fps):
        self.fps = fps
        self.timer.start(int(1000/self.fps))

    @staticmethod
    def _on_destroyed(self):
        """Dubbel check to kill timer"""
        self.timer.stop()


# -----------------------------------------------------
#    END OF 3D-RENDERING
# -----------------------------------------------------
class QtSliders(QWidget):
    """Class to handle sliders"""
    def __init__(self, view):
        super(QWidget, self).__init__()
        hor_box = QHBoxLayout()
        self.setWindowTitle("FPS")
        self.view3D = view
        self.label = QLabel()
        self.label.setText("fps: " + str(self.view3D.fps))

        # Create slider for editing FPS
        self.fps_slider = QSlider()
        self.fps_slider.setMinimum(1)
        self.fps_slider.setMaximum(300)
        self.fps_slider.setValue(self.view3D.fps)
        self.fps_slider.setTickPosition(QSlider.TicksBelow)
        self.fps_slider.setTickInterval(10)
        hor_box.addWidget(self.label)
        hor_box.addWidget(self.fps_slider)
        self.setLayout(hor_box)
        self.fps_slider.valueChanged.connect(self.slider_changed)

    def slider_changed(self):
        """Update FPS value in render class"""
        fps = self.fps_slider.value()
        self.view3D.update_timer(fps)
        self.label.setText("fps: " + str(self.view3D.fps))


class QtLabel(QWidget):
    """Class to handle the printing of matrices labels"""
    def __init__(self, view):
        super(QWidget, self).__init__()
        self.view3D = view
        vbox = QVBoxLayout()
        self.fps = 30
        self.label = QLabel(self)
        self.label.setText("")
        self.label.setFont(QtGui.QFont("Times", 12))
        vbox.addWidget(self.label)
        self.setLayout(vbox)
        self.timer = QTimer()
        self.start()

    def start(self):
        """"Starts timer to call window update"""
        self.timer.timeout.connect(self.update_window)
        self.timer.start(int(1000/self.fps))

    def update_window(self):
        """update label texts"""
        text = str(self.view3D.r1)
        self.label.setText(text)


# ------------------------------------------------------
# MAIN
# ------------------------------------------------------
if __name__ == "__main__":
    def run_app():
        # Create framework
        main_win = QtMain("Robot Modelling V4.0.1")
        np.set_printoptions(precision=1, suppress=True)
        # Add 3d renderer and label to framework
        view_3d = RenderWindow(main_win)
        label = QtLabel(view_3d)
        main_win.hbox.addWidget(view_3d, 3)
        main_win.hbox.addWidget(label, 1)
        # Add popup window to framework
        main_win.pop = QtSliders(view_3d)
        sys.exit(main_win.app.exec_())


    run_app()
