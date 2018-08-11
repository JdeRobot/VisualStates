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
             Pushkal Katara (katarapushkal@gmail.com)

  '''

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QDialog, QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication, QLabel, QComboBox, \
    QFormLayout, QTabWidget, QPlainTextEdit, QInputDialog, QFileDialog, QMessageBox

class NamespaceDialog(QDialog):
    namespaceChanged = pyqtSignal()

    def __init__(self, name, namespace):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.tabWidget = None

        self.namespace = namespace

        self.tabWidget = QTabWidget()
        mainLayout = QFormLayout()
        mainLayout.addWidget(self.tabWidget)

        self.functionTab = FunctionTab()
        self.functionTab.namespaceChanged.connect(self.namespaceChangedHandler)
        self.tabWidget.addTab(self.functionTab, 'Functions')
        self.functionTab.setFunctions(namespace)

        self.variableTab = VariableTab()
        self.variableTab.namespaceChanged.connect(self.namespaceChangedHandler)
        self.tabWidget.addTab(self.variableTab, 'Variables')
        self.variableTab.setVariables(namespace)

        self.resize(700, 100)
        self.setLayout(mainLayout)

    def setNamespace(self, namespace):
        self.namespace = namespace

    def namespaceChangedHandler(self):
        self.namespaceChanged.emit()

    def getNamespace(self):
        return self.namespace

class FunctionTab(QWidget):
    namespaceChanged = pyqtSignal()
    def __init__(self):
        super(QWidget, self).__init__()
        layout = QFormLayout()
        self.setLayout(layout)

        self.functions = QPlainTextEdit()
        self.functions.textChanged.connect(self.functionsChanged)
        self.functions.setMinimumHeight(800)
        layout.addRow('Functions', self.functions)

    def functionsChanged(self):
        if self.namespace:
            self.namespace.setFunctions(self.functions.toPlainText())
            self.namespaceChanged.emit()

    def setFunctions(self, namespace):
        self.namespace = namespace
        self.functions.setPlainText(self.namespace.getFunctionsAsText())

class VariableTab(QWidget):
    namespaceChanged = pyqtSignal()
    def __init__(self):
        super(QWidget, self).__init__()
        layout = QFormLayout()
        self.setLayout(layout)

        self.variables = QPlainTextEdit()
        self.variables.textChanged.connect(self.variablesChanged)
        self.variables.setMinimumHeight(800)
        layout.addRow('Variables', self.variables)

    def variablesChanged(self):
        if self.namespace:
            self.namespace.setVariables(self.variables.toPlainText())
            self.namespaceChanged.emit()

    def setVariables(self, namespace):
        self.namespace = namespace
        self.variables.setPlainText(self.namespace.getVariablesAsText())
