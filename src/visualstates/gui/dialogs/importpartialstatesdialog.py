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
    QApplication, \
    QTreeWidgetItem
from PyQt5.QtCore import pyqtSignal, \
    Qt

class ImportPartialStatesDialog(QDialog):
    fileChanged = pyqtSignal(list)

    def __init__(self, name, file):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.setFixedSize(400, 600)
        self.file = file
        self.list = []

        self.drawWindow()

    def drawWindow(self):
        self.treeWidget = QTreeWidget(self)
        headerItem = QTreeWidgetItem()
        headerItem.setText(0, "Select States")
        self.treeWidget.setHeaderItem(headerItem)
        self.treeWidget.resize(400, 560)
        self.loadStates(self.treeWidget, self.file[0])

        self.button = QPushButton(self)
        self.button.setText("Ok")
        self.button.move(310, 567)
        self.button.clicked.connect(self.buttonClicked)

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
        for child in parentState.getChildren():
            if self.list[child.id - 1].checkState(0) == Qt.Unchecked:
                parentState.removeChild(child)
            else:
                self.removeStates(child)

    def buttonClicked(self, event):
        self.removeStates(self.file[0])
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = ImportPartialStatesDialog('Select States', ['hmm'])
    dialog.exec_()