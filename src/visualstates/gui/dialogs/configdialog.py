'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import sys

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtWidgets import QDialog, QApplication, QGroupBox, QComboBox, \
    QVBoxLayout, QFormLayout

from visualstates.configs.config import ROS, RosConfig
from visualstates.gui.dialogs.rosconfigdialog import RosConfigDialog


class ConfigDialog(QDialog):
    configChanged = pyqtSignal()
    def __init__(self, title, config):
        QDialog.__init__(self)
        if config is not None:
            self.type = config.type
        else:
            self.type = ROS

        self.setWindowTitle(title)
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)

        vLayout = QFormLayout()

        self.configsLayout = QVBoxLayout()
        self.configsBox = QGroupBox('')
        self.configsBox.setLayout(self.configsLayout)
        vLayout.addWidget(self.configsBox)
        self.setLayout(vLayout)
        self.resize(700, 500)

        self.rosConfigsUI = RosConfigDialog('ROS Communication')
        self.rosConfigsUI.configChanged.connect(self.configChangedHandler)
        self.configsLayout.addWidget(self.rosConfigsUI)
        self.rosConfigsUI.setVisible(True)
        self.rosConfig = None

        if config is not None:
            if config.type == ROS:
                self.rosConfig = config
                # self.commTypeCombo.setCurrentIndex(1)
                self.loadRosConfigs()
        else:
            self.loadRosConfigs()

    def commTypeComboChanged(self):
        if self.commTypeCombo.currentData() == 'ros':
            self.loadRosConfigs()

    def loadRosConfigs(self):
        self.type = ROS
        if self.rosConfig is None:
            self.rosConfig = RosConfig()
        self.rosConfigsUI.setConfig(self.rosConfig)
        self.configChanged.emit()

    def configChangedHandler(self):
        self.configChanged.emit()

    def getConfig(self):
        if self.type == ROS:
            return self.rosConfig
        # elif self.type == JDEROBOTCOMM:
        #     return self.jdeRobotCommConfig

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # config = JdeRobotConfig()
    config = RosConfig()
    dialog = ConfigDialog('Config', config)
    dialog.exec_()
