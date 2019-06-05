# from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QMainWindow, QLabel, QGridLayout, QWidget, QVBoxLayout, QHBoxLayout, QApplication
from PyQt5.QtCore import QSize, QRect
import sys

# ----------------------------------------
# Python Qt framework
# Cédric Wassenaar
# c.m.wassenaar@student.hhs.nl
# 24-09-2018
# See below for how to use
# -----------------------------------------

__version__ = '2018.9.10'
__docformat__ = 'restructuredtext en'


class QtMain(QMainWindow):
    """Main frame class"""
    def __init__(self, name):
        self.app = QApplication(sys.argv)
        QMainWindow.__init__(self)
        self.setMinimumSize(QSize(1280, 720))
        self.setWindowTitle(name)
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        self.hbox = QHBoxLayout(self)
        central_widget.setLayout(self.hbox)
        menu = self.menuBar().addMenu('Settings')
        quit_action = menu.addAction('Exit')
        quit_action.triggered.connect(QApplication.quit)
        menu = self.menuBar().addMenu('Preferences')
        fps_option = menu.addAction('Change FPS')
        fps_option.triggered.connect(self.openPopup)
        screen_option = menu.addAction('Fullscreen/Windowed')
        screen_option.triggered.connect(self.switchWindow)
        self.pop = QWidget()
        self.show()

    def openPopup(self):
        # Method to call popup
        self.pop.setGeometry(QRect(100, 100, 200, 300))
        self.pop.show()

    def switchWindow(self):
        """Switches window to fullscreen or windowed"""
        self.close()
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()


"""
Below an example is given how to use the QtMain framework

def run_app():
    mainWindow = WinMain()

    #Add a new widget to 
    mainWindow.hbox.addWidget(<QWidget object 1>, <Size ratio e.g. 3>)
    mainWindow.hbox.addWidget(<QWidget object 2>, <Size ratio e.g. 1>)

    #Add popup screen (Currently only 1 possible)
    mainWindow.pop = <QWidget object 3>

    sys.exit(mainWindow.app.exec_())
"""