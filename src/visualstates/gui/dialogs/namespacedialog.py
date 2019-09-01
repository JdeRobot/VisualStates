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

from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QLabel, QRadioButton, \
    QTabWidget, QScrollArea, QGroupBox, QBoxLayout
from PyQt5.QtGui import QFontDatabase, QColor, QFontMetrics
from PyQt5.Qsci import QsciScintilla, QsciLexerPython, QsciLexerCPP
from visualstates.gui.dialogs.paramprop import ParamPropDialog
from visualstates.gui.tools.elidedlabel import ElidedLabel
from collections import OrderedDict

class NamespaceDialog(QDialog):
    namespaceChanged = pyqtSignal()

    def __init__(self, name, namespace, globalNamespace=False):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.resize(820,600)

        self.namespace = namespace

        mainLayout = QVBoxLayout()

        self.pythonButton = QRadioButton('Python')
        self.pythonButton.setChecked(True)
        self.pythonButton.clicked.connect(self.pythonClicked)
        self.cppButton = QRadioButton('C++')
        self.cppButton.clicked.connect(self.cppClicked)

        hLayout0 = QHBoxLayout()
        hLayout0.addWidget(self.pythonButton)
        hLayout0.addWidget(self.cppButton)
        container0 = QWidget()
        container0.setLayout(hLayout0)
        mainLayout.addWidget(container0)

        self.language = 'python'

        self.tabWidget = QTabWidget()

        self.functionTab = FunctionsTab(self.namespace.getFunctions())
        self.functionTab.functionsChanged.connect(self.functionsChanged)
        self.tabWidget.addTab(self.functionTab, 'Functions')

        self.variableTab = VariablesTab(self.namespace.getVariables())
        self.variableTab.variablesChanged.connect(self.variablesChanged)
        self.tabWidget.addTab(self.variableTab, 'Variables')

        if not globalNamespace:
            self.paramTab = ParamsTab(self.namespace.getParams())
            self.paramTab.paramsChanged.connect(self.paramsChanged)
            self.tabWidget.addTab(self.paramTab, 'Parameters')

        mainLayout.addWidget(self.tabWidget)

        self.setLayout(mainLayout)

    def functionsChanged(self, functions):
        self.namespace.setFunctions(functions)
        self.namespaceChanged.emit()

    def variablesChanged(self, variables):
        self.namespace.setVariables(variables)
        self.namespaceChanged.emit()

    def paramsChanged(self, params):
        self.namespace.setParams(params)
        self.namespaceChanged.emit()

    def setNamespace(self, namespace):
        self.namespace = namespace

    def getNamespace(self):
        return self.namespace

    def pythonClicked(self):
        #TODO
        pass

    def cppClicked(self):
        #TODO
        pass

class FunctionsTab(QDialog):
    functionsChanged = pyqtSignal('QString')

    def __init__(self, functions):
        super(QDialog, self).__init__()

        self.codeDialog = CodeDialog(functions)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeDialog)
        self.setLayout(verticalLayout)

        self.codeDialog.codeChanged.connect(self.changeFunctions)

    def changeFunctions(self, functions):
        self.functionsChanged.emit(functions)


class VariablesTab(QDialog):
    variablesChanged = pyqtSignal('QString')

    def __init__(self, variables):
        super(QDialog, self).__init__()

        self.codeDialog = CodeDialog(variables)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeDialog)
        self.setLayout(verticalLayout)

        self.codeDialog.codeChanged.connect(self.changeVariables)

    def changeVariables(self, variables):
        self.variablesChanged.emit(variables)

class CodeDialog(QDialog):
    codeChanged = pyqtSignal('QString')

    def __init__(self, code):
        super(QDialog, self).__init__()

        self.codeEdit = QsciScintilla()
        self.codeEdit.setText(code)
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        fontmetrics = QFontMetrics(fixedWidthFont)
        self.codeEdit.setMarginWidth(0, fontmetrics.width("000"))
        self.codeEdit.setMarginLineNumbers(0, True)
        self.codeEdit.setMarginsBackgroundColor(QColor("#cccccc"))

        self.codeEdit.setBraceMatching(QsciScintilla.SloppyBraceMatch)
        self.codeEdit.setCaretLineVisible(True)
        self.codeEdit.setCaretLineBackgroundColor(QColor("#ffe4e4"))
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.codeEdit.SendScintilla(QsciScintilla.SCI_SETHSCROLLBAR, 0)
        self.codeEdit.setUtf8(True)

        self.codeEdit.setTabWidth(4)
        self.codeEdit.setIndentationsUseTabs(True)
        self.codeEdit.setIndentationGuides(True)
        self.codeEdit.setTabIndents(True)
        self.codeEdit.setAutoIndent(True)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeEdit)
        self.setLayout(verticalLayout)

        self.codeEdit.textChanged.connect(self.changeCode)

    def changeCode(self):
        self.codeChanged.emit(self.codeEdit.text())

class ParamsTab(QDialog):
    paramsChanged = pyqtSignal(list)

    def __init__(self, params):
        super(QDialog, self).__init__()
        self.params = OrderedDict()
        for param in params:
            self.params[param.name] = param
        self.paramUIs = {}
        self.removeIds = []
        self.drawWindow()

    def drawWindow(self):
        VLayout = QVBoxLayout()

        btnLayout = QHBoxLayout()
        btnLayout.setAlignment(Qt.AlignLeft)
        newBtn = QPushButton("New")
        newBtn.setFixedWidth(200)
        newBtn.clicked.connect(self.newClicked)
        btnLayout.addWidget(newBtn)
        VLayout.addLayout(btnLayout)

        rowLayout = QHBoxLayout()

        rowLayout.setAlignment(Qt.AlignLeft)
        rowLayout.addSpacing(10)
        titleLblStyleSheet = 'QLabel {font-weight: bold;}'
        nameLbl = QLabel('Name')
        nameLbl.setStyleSheet(titleLblStyleSheet)
        nameLbl.setFixedWidth(100)
        rowLayout.addWidget(nameLbl)
        typeLbl = QLabel('Type')
        typeLbl.setStyleSheet(titleLblStyleSheet)
        typeLbl.setFixedWidth(60)
        rowLayout.addWidget(typeLbl)
        valueLbl = QLabel('Value')
        valueLbl.setStyleSheet(titleLblStyleSheet)
        valueLbl.setFixedWidth(100)
        rowLayout.addWidget(valueLbl)
        descLbl = QLabel('Description')
        descLbl.setStyleSheet(titleLblStyleSheet)
        descLbl.setMinimumWidth(280)
        rowLayout.addWidget(descLbl)

        VLayout.addLayout(rowLayout)

        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        #scrollArea.setStyleSheet('QScrollArea {border: 0px;}')
        self.scrollVlayout = QVBoxLayout()
        self.scrollVlayout.setDirection(QBoxLayout.TopToBottom)
        self.scrollVlayout.setAlignment(Qt.AlignTop)

        dummyBox = QGroupBox()
        dummyBox.setStyleSheet('QGroupBox {padding-top: 20px; margin-top: -20px;}')
        dummyBox.setLayout(self.scrollVlayout)
        scrollArea.setWidget(dummyBox)
        VLayout.addWidget(scrollArea)

        for param in self.params.values():
            self.addParam(param)
        self.setLayout(VLayout)

    def addParam(self, param):
        rowLayout = QHBoxLayout()
        nameLbl = ElidedLabel(param.name)
        nameLbl.setToolTip(param.name)
        nameLbl.setFixedWidth(100)
        rowLayout.addWidget(nameLbl)
        typeLbl = ElidedLabel(param.type)
        typeLbl.setFixedWidth(60)
        rowLayout.addWidget(typeLbl)
        valueLbl = ElidedLabel(param.value)
        valueLbl.setToolTip(param.value)
        valueLbl.setFixedWidth(100)
        rowLayout.addWidget(valueLbl)
        descLbl = ElidedLabel(param.desc)
        descLbl.setAlignment(Qt.AlignTop)
        descLbl.setFixedHeight(17)
        descLbl.setToolTip(param.desc)
        descLbl.setMinimumWidth(280)
        rowLayout.addWidget(descLbl)

        editBtn = QPushButton('Edit')
        editBtn.setFixedWidth(80)
        editBtn.setObjectName(param.name)
        editBtn.clicked.connect(self.editHandler)
        rowLayout.addWidget(editBtn)
        removeBtn = QPushButton('Remove')
        removeBtn.setFixedWidth(80)
        removeBtn.setObjectName(param.name)
        removeBtn.clicked.connect(self.removeHandler)
        rowLayout.addWidget(removeBtn)

        self.scrollVlayout.addLayout(rowLayout)
        UI = [nameLbl, typeLbl, valueLbl, descLbl, editBtn, removeBtn]
        self.paramUIs[param.name] = UI

    def newClicked(self):
        dialog = ParamPropDialog(params=self.params)
        dialog.paramAdded.connect(self.paramAddedHandler)
        dialog.exec_()

    def paramAddedHandler(self, params, name):
        self.params = params
        self.emitParams()
        self.addParam(self.params[name])

    def removeHandler(self):
        removeId = self.sender().objectName()
        self.params.pop(removeId, None)
        self.emitParams()
        for uiItem in self.paramUIs[removeId]:
            uiItem.deleteLater()


    def editHandler(self):
        editID = self.sender().objectName()
        dialog = ParamPropDialog(params=self.params, id=editID)
        dialog.paramUpdated.connect(self.paramUpdatedHandler)
        dialog.exec_()

    def paramUpdatedHandler(self, params, id):
        self.params = params
        self.emitParams()
        param = self.params[id]
        UI = self.paramUIs[id]
        UI[0].setText(param.name)
        UI[0].setToolTip(param.name)
        UI[1].setText(param.type)
        UI[1].setToolTip(param.type)
        UI[2].setText(param.value)
        UI[2].setToolTip(param.value)
        UI[3].setText(param.desc)
        UI[3].setToolTip(param.desc)
        self.params.pop(id, None)
        self.params[param.name] = param
        self.paramUIs.pop(id, None)
        self.paramUIs[param.name] = UI

    def emitParams(self):
        params = []
        for param in self.params.values():
            params.append(param)
        self.paramsChanged.emit(params)