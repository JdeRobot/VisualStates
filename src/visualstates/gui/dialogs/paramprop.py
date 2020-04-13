'''
   Copyright (C) 1997-2019 JDERobot Developers Team

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
from PyQt5.QtWidgets import QDialog, QLabel, QLineEdit, \
    QPushButton, QApplication, QHBoxLayout, QVBoxLayout, \
    QScrollArea, QComboBox, QGridLayout, QPlainTextEdit, \
    QMessageBox
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import *
from visualstates.core.parameter import Parameter, isParamName, isTypeEqualValue

class ParamPropDialog(QDialog):

    paramAdded = pyqtSignal(dict, str)
    paramUpdated = pyqtSignal(dict, str)

    def __init__(self, param=None, params=None, id="", modify=True):
        super(QDialog, self).__init__()
        self.params = params
        self.id = id
        self.param = Parameter()
        self.setFixedSize(400, 360)
        if not modify:
            self.setWindowTitle('View Parameter')
            self.param = param
        elif id == "":
            self.setWindowTitle('Add Parameter')
        else:
            self.setWindowTitle('Edit Parameter')
            self.param = self.params[self.id]

        Layout = QVBoxLayout()
        self.setLayout(Layout)
        rowLayout = QHBoxLayout()
        nameLbl = QLabel('Name :')
        nameLbl.setFixedWidth(100)
        rowLayout.addWidget(nameLbl)
        self.nameEdit = QLineEdit(self.param.name)
        rowLayout.addWidget(self.nameEdit)
        Layout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        typeLbl = QLabel('Type :')
        typeLbl.setFixedWidth(100)
        rowLayout.addWidget(typeLbl)
        if modify:
            self.typeCb = QComboBox()
            self.typeCb.addItems(['String', 'Character', 'Integer', 'Float', 'Boolean'])
            self.typeCb.setCurrentText(self.param.type)
            rowLayout.addWidget(self.typeCb)
        else:
            self.typeEdit = QLineEdit(self.param.type)
            rowLayout.addWidget(self.typeEdit)
        Layout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        valueLbl = QLabel('Value :')
        valueLbl.setFixedWidth(100)
        rowLayout.addWidget(valueLbl)
        self.valueEdit = QLineEdit(self.param.value)
        rowLayout.addWidget(self.valueEdit)
        Layout.addLayout(rowLayout)

        rowLayout = QHBoxLayout()
        descLbl = QLabel('Description :')
        descLbl.setFixedWidth(100)
        descLbl.setAlignment(Qt.AlignTop)
        rowLayout.addWidget(descLbl)
        self.descEdit = QPlainTextEdit(self.param.desc)
        self.descEdit.setFixedHeight(200)
        rowLayout.addWidget(self.descEdit)
        Layout.addLayout(rowLayout)

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignRight)
        if modify:
            saveBtn = QPushButton("Save")
            saveBtn.setFixedWidth(80)
            saveBtn.clicked.connect(self.saveClicked)
            btnLayout.addWidget(saveBtn)
        closeBtn = QPushButton("Close")
        closeBtn.setFixedWidth(80)
        closeBtn.clicked.connect(self.closeClicked)
        btnLayout.addWidget(closeBtn)
        Layout.addLayout(btnLayout)
        self.setLayout(Layout)

        if not modify:
            self.nameEdit.setReadOnly(True)
            self.typeEdit.setReadOnly(True)
            self.valueEdit.setReadOnly(True)
            self.descEdit.setReadOnly(True)

    def saveClicked(self):
        if not isTypeEqualValue(self.typeCb.currentText(), self.valueEdit.text().strip()):
            QMessageBox.warning(self, 'Error', 'Value is not same as type')
        elif not isParamName(self.nameEdit.text().strip()):
            QMessageBox.warning(self, 'Error', 'Name does not meet requirements of a parameter.\n\nIt must start with an alphabet and can only contain alphanumeric characters and underscores.')
        else:
            if self.nameEdit.text() != self.id and self.id != "":
                for param in self.params.values():
                    if param.name == self.nameEdit.text():
                        QMessageBox.warning(self, 'Error', 'Name already in use')
                        return
            newParam = False
            if self.id == "":
                newParam = True
            self.param.setName(self.nameEdit.text().strip())
            self.param.setType(self.typeCb.currentText())
            self.param.setValue(self.valueEdit.text().strip())
            self.param.setDesc(self.descEdit.toPlainText())
            if newParam:
                self.params[self.param.name] = self.param
                self.paramAdded.emit(self.params, self.param.name)
            else:
                self.paramUpdated.emit(self.params, self.id)
            self.accept()

    def closeClicked(self):
        self.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = ParamPropDialog(params=[])
    dialog.exec_()