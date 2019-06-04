'''
     Copyright (C) 1997-2018 JDERobot Developers Team

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

     Authors : Baidyanath Kundu (kundubaidya99@gmail.com)

  '''

import sys
from PyQt5.QtWidgets import QDialog, \
    QPushButton, QTreeWidget, \
    QApplication, QGridLayout, \
    QTreeWidgetItem, QLabel
from PyQt5.QtCore import pyqtSignal, \
    Qt

class SelectStatesDialog(QDialog):
    fileChanged = pyqtSignal(list)

    def __init__(self, name, file):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.setFixedSize(400, 600)
        self.file = file
        self.list = []

        self.drawWindow()

    def drawWindow(self):
        gridLayout = QGridLayout()

        gridLayout.addWidget(QLabel("Select States"), 0, 0)

        self.treeWidget = QTreeWidget(self)
        gridLayout.addWidget(self.treeWidget, 1, 0)
        headerItem = QTreeWidgetItem()
        headerItem.setText(0, "Name")
        self.treeWidget.setHeaderItem(headerItem)
        self.loadStates(self.treeWidget, self.file[0])

        self.button = QPushButton(self)
        self.button.setText("Ok")
        gridLayout.addWidget(self.button, 2, 0, alignment=Qt.AlignRight)
        self.button.clicked.connect(self.buttonClicked)

        self.setLayout(gridLayout)

    def loadStates(self, parentItem, parentState):
        for child in parentState.getChildren():
            if type(parentItem) == QTreeWidgetItem:
                parentItem.setFlags(parentItem.flags() | Qt.ItemIsTristate)
            item = QTreeWidgetItem(parentItem)
            item.setText(0, child.name)
            item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            item.setCheckState(0, Qt.Checked)
            if len(self.list) < child.id:
                self.list.extend([None] * (child.id - len(self.list)))
            self.list[child.id - 1] = item
            self.loadStates(item, child)

    def removeStates(self, parentState):
        remList = []
        for child in parentState.getChildren():
            if self.list[child.id - 1].checkState(0) == Qt.Unchecked:
                remList.append(child)
            else:
                self.removeStates(child)
        for child in remList:
            for transition in child.getDestTransitions():
                transition.origin.removeOriginTransition(transition)
            parentState.removeChild(child)

    def buttonClicked(self, event):
        self.removeStates(self.file[0])
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = SelectStatesDialog('Select States', ['hmm'])
    dialog.exec_()