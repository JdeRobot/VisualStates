# -*- coding: utf-8 -*-
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
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPixmap
from PyQt5.QtWidgets import QMainWindow, QAction, QDockWidget, QTreeView, QGraphicsView, \
    QWidget, QFileDialog, QLabel, QVBoxLayout, QPushButton, QMessageBox

from gui.automata.automatascene import AutomataScene, OpType
from parsers.filemanager import FileManager
from parsers.importmanager import ImportManager
from gui.tree.treemodel import TreeModel
from core.state import State
from core.namespace import Namespace
from gui.transition.timerdialog import TimerDialog
from gui.state.codedialog import CodeDialog
from gui.dialogs.librariesdialog import LibrariesDialog
from gui.dialogs.configdialog import ConfigDialog
from generators.cppgenerator import CppGenerator
from generators.pythongenerator import PythonGenerator
from configs.interfaces import Interfaces
from gui.cmakevars import CMAKE_INSTALL_PREFIX
from configs.config import JdeRobotConfig, RosConfig, ROS, JDEROBOTCOMM
from generators.cpprosgenerator import CppRosGenerator
from generators.pythonrosgenerator import PythonRosGenerator

class VisualStates(QMainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__()

        self.setWindowTitle("VisualStates")
        self.configDialog = None

        # root state
        self.rootState = State(0, "root", True)
        self.activeState = self.rootState
        self.activeNamespace = Namespace(0, "root", None, None)

        # create status bar
        self.statusBar()

        self.createMenu()
        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.fileManager = FileManager()
        self.importManager = ImportManager()

        self.automataPath = None

        self.libraries = []
        self.config = None
        self.namespaces = []
        self.namespaces.append(self.activeNamespace)
        self.interfaceHeaderMap = Interfaces.getInterfaces()

    def createMenu(self):
        # create actions
        newAction = QAction('&New', self)
        newAction.setShortcut('Ctrl+N')
        newAction.setStatusTip('Create New Visual States')
        newAction.triggered.connect(self.newAction)

        openAction = QAction('&Open', self)
        openAction.setShortcut('Ctrl+O')
        openAction.setStatusTip('Open Visual States')
        openAction.triggered.connect(self.openAction)

        importAction = QAction('&Import', self)
        openAction.setShortcut('Ctrl+I')
        importAction.setStatusTip('Import A State')
        importAction.triggered.connect(self.importAction)

        saveAction = QAction('&Save', self)
        saveAction.setShortcut('Ctrl+S')
        saveAction.setStatusTip('Save Visual States')
        saveAction.triggered.connect(self.saveAction)

        saveAsAction = QAction('&Save As', self)
        saveAsAction.setShortcut('Ctrl+S')
        saveAsAction.setStatusTip('Save Visual States as New One')
        saveAsAction.triggered.connect(self.saveAsAction)

        quitAction = QAction('&Quit', self)
        quitAction.setShortcut('Ctrl+Q')
        quitAction.setStatusTip('Quit Visual States')
        quitAction.triggered.connect(self.quitAction)

        # figures menu
        stateAction = QAction('&State', self)
        stateAction.setStatusTip('Create a state')
        stateAction.triggered.connect(self.stateAction)

        transitionAction = QAction('&Transition', self)
        transitionAction.setStatusTip('Create a transition')
        transitionAction.triggered.connect(self.transitionAction)

        # data menu
        timerAction = QAction('&Timer', self)
        timerAction.setShortcut('Ctrl+M')
        timerAction.setStatusTip('Set timing of states')
        timerAction.triggered.connect(self.timerAction)

        variablesAction = QAction('&Variables', self)
        variablesAction.setShortcut('Ctrl+V')
        variablesAction.setStatusTip('Define state variables')
        variablesAction.triggered.connect(self.variablesAction)

        functionsAction = QAction('&Functions', self)
        functionsAction.setShortcut('Ctrl+F')
        functionsAction.setStatusTip('Define functions')
        functionsAction.triggered.connect(self.functionsAction)

        # actions menu
        librariesAction = QAction('&Libraries', self)
        librariesAction.setShortcut('Ctrl+L')
        librariesAction.setStatusTip('Add additional libraries')
        librariesAction.triggered.connect(self.librariesAction)

        configFileAction = QAction('&Config File', self)
        configFileAction.setShortcut('Ctrl+C')
        configFileAction.setStatusTip('Edit configuration file')
        configFileAction.triggered.connect(self.configFileAction)

        generateCppAction = QAction('&Generate C++', self)
        generateCppAction.setShortcut('Ctrl+G')
        generateCppAction.setStatusTip('Generate C++ code')
        generateCppAction.triggered.connect(self.generateCppAction)

        # compileCppAction = QAction('&Compile C++', self)
        # compileCppAction.setShortcut('Ctrl+P')
        # compileCppAction.setStatusTip('Compile generated C++ code')
        # compileCppAction.triggered.connect(self.compileCppAction)

        generatePythonAction = QAction('&Generate Python', self)
        generatePythonAction.setShortcut('Ctrl+Y')
        generatePythonAction.setStatusTip('Generate Python code')
        generatePythonAction.triggered.connect(self.generatePythonAction)

        # help menu
        aboutAction = QAction('&About', self)
        aboutAction.setShortcut('F1')
        aboutAction.setStatusTip('Information about VisualStates')
        aboutAction.triggered.connect(self.aboutAction)

        # create main menu
        menubar = self.menuBar()
        archieveMenu = menubar.addMenu('&File')
        archieveMenu.addAction(newAction)
        archieveMenu.addAction(openAction)
        archieveMenu.addAction(importAction)
        archieveMenu.addAction(saveAction)
        archieveMenu.addAction(saveAsAction)
        archieveMenu.addAction(quitAction)

        figuresMenu = menubar.addMenu('&Figures')
        figuresMenu.addAction(stateAction)
        figuresMenu.addAction(transitionAction)

        dataMenu = menubar.addMenu('&Data')
        dataMenu.addAction(timerAction)
        dataMenu.addAction(variablesAction)
        dataMenu.addAction(functionsAction)

        actionsMenu = menubar.addMenu('&Actions')
        actionsMenu.addAction(librariesAction)
        actionsMenu.addAction(configFileAction)
        actionsMenu.addAction(generateCppAction)
        # actionsMenu.addAction(compileCppAction)
        actionsMenu.addAction(generatePythonAction)

        helpMenu = menubar.addMenu('&Help')
        helpMenu.addAction(aboutAction)


    def newAction(self):
        self.automataScene.clearScene()
        self.treeModel.removeAll()

        # create new root state
        self.rootState = State(0, 'root', True)
        self.activeNamespace = Namespace(0, "root", None, None)

        self.automataScene.setActiveState(self.rootState)
        self.automataScene.resetIndexes()

        self.libraries = []
        self.config = None
        self.namespaces = []

    def openAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Open VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptOpen)
        if fileDialog.exec_():
            (self.rootState, self.config, self.libraries, self.namespaces) = self.fileManager.open(fileDialog.selectedFiles()[0])
            # Find root namespace and set it.
            for namespace in self.namespaces:
                if(namespace.id == unicode(0) and namespace.name == "root"):
                    self.activeNamespace = namespace
                    
            self.automataPath = self.fileManager.fullPath
            self.treeModel.removeAll()
            self.treeModel.loadFromRoot(self.rootState)
            # set the active state as the loaded state
            self.automataScene.setActiveState(self.rootState)
            self.automataScene.setLastIndexes(self.rootState)

            # print(str(self.config))
        # else:
        #     print('open is canceled')

    def saveAction(self):
        if len(self.fileManager.getFileName()) == 0:
            self.saveAsAction()
        else:
            self.fileManager.save(self.rootState, self.config, self.libraries, self.namespaces)

    def saveAsAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Save VisualStates Project")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setAcceptMode(QFileDialog.AcceptSave)
        if fileDialog.exec_():
            self.fileManager.setFullPath(fileDialog.selectedFiles()[0])
            self.fileManager.save(self.rootState, self.config, self.libraries, self.namespaces)
        # else:
        #     print('file dialog canceled')

    def quitAction(self):
        # print('Quit')
        self.close()

    def stateAction(self):
        self.automataScene.setOperationType(OpType.ADDSTATE)

    def transitionAction(self):
        self.automataScene.setOperationType(OpType.ADDTRANSITION)

    def importAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Import VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptOpen)
        if fileDialog.exec_():
            file = self.fileManager.open(fileDialog.selectedFiles()[0])
            self.fileManager.setPath(self.automataPath)
            # Update importing Namespaces
            importedState, self.config, self.libraries, self.namespaces = self.importManager.updateAuxiliaryData(file, self)
            self.treeModel.loadFromRoot(importedState, self.activeState)
            self.automataScene.displayState(self.activeState)
            self.automataScene.setLastIndexes(self.rootState)

    def timerAction(self):
        if self.activeState is not None:
            timerDialog = TimerDialog('Time Step Duration', str(self.activeState.getTimeStep()))
            timerDialog.timeChanged.connect(self.timeStepDurationChanged)
            timerDialog.exec_()

    def variablesAction(self):
        variablesDialog = CodeDialog('Variables', str(self.activeNamespace.variables))
        variablesDialog.codeChanged.connect(self.variablesChanged)
        variablesDialog.exec_()

    def functionsAction(self):
        functionsDialog = CodeDialog('Functions', self.activeNamespace.functions)
        functionsDialog.codeChanged.connect(self.functionsChanged)
        functionsDialog.exec_()

    def librariesAction(self):
        librariesDialog = LibrariesDialog('Libraries', self.libraries)
        librariesDialog.librariesChanged.connect(self.librariesChanged)
        librariesDialog.exec_()

    def configFileAction(self):
        self.configDialog = ConfigDialog('Config', self.config)
        self.configDialog.configChanged.connect(self.configChanged)
        self.configDialog.exec_()

    def showWarning(self, title, msg):
        QMessageBox.warning(self, title, msg)

    def showInfo(self, title, msg):
        QMessageBox.information(self, title, msg)

    def generateCppAction(self):
        stateList = []
        if self.fileManager.hasFile():
            self.getStateList(self.rootState, stateList)
            if self.config.type == ROS:
                generator = CppRosGenerator(self.libraries, self.config, self.interfaceHeaderMap, stateList, self.namespaces)
            elif self.config.type == JDEROBOTCOMM:
                generator = CppGenerator(self.libraries, self.config, self.interfaceHeaderMap, stateList, self.namespaces)

            generator.generate(self.fileManager.getPath(), self.fileManager.getFileName())
            self.showInfo('C++ Code Generation', 'C++ code generation is successful.')
        else:
            self.showWarning('C++ Generation', 'Please save the project before code generation.')


    # def compileCppAction(self):
    #     # print('compile cpp action')
    #     pass

    def generatePythonAction(self):
        stateList = []
        if self.fileManager.hasFile():
            self.getStateList(self.rootState, stateList)
            if self.config.type == ROS:
                generator = PythonRosGenerator(self.libraries, self.config, stateList, self.namespaces)
            elif self.config.type == JDEROBOTCOMM:
                generator = PythonGenerator(self.libraries, self.config, self.interfaceHeaderMap, stateList, self.namespaces)
            generator.generate(self.fileManager.getPath(), self.fileManager.getFileName())
            self.showInfo('Python Code Generation', 'Python code generation is successful.')
        else:
            self.showWarning('Python Generation', 'Please save the project before code generation.')

    def aboutAction(self):
        pass
        # print('about action')

    def createTreeView(self):
        dockWidget = QDockWidget()
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setTitleBarWidget(QWidget())
        self.treeView = QTreeView()
        self.treeView.clicked.connect(self.treeItemClicked)
        self.treeModel = TreeModel()
        self.treeView.setModel(self.treeModel)

        self.logo = QLabel()
        logoPixmap = QPixmap(CMAKE_INSTALL_PREFIX + '/share/resources/jderobot.png')
        self.logo.setPixmap(logoPixmap)

        self.upButton = QPushButton()
        self.upButton.setText('Up')
        self.upButton.clicked.connect(self.upButtonClicked)

        leftContainer = QWidget()
        leftLayout = QVBoxLayout()
        leftLayout.addWidget(self.treeView)
        leftLayout.addWidget(self.upButton)
        leftLayout.addWidget(self.logo)
        leftContainer.setLayout(leftLayout)

        dockWidget.setWidget(leftContainer)
        self.addDockWidget(Qt.LeftDockWidgetArea, dockWidget)

    def createStateCanvas(self):
        self.stateCanvas = QGraphicsView()
        self.automataScene = AutomataScene()
        self.automataScene.setSceneRect(0, 0, 2000, 2000)
        self.automataScene.activeStateChanged.connect(self.activeStateChanged)
        self.automataScene.stateInserted.connect(self.stateInserted)
        self.automataScene.stateRemoved.connect(self.stateRemoved)
        self.automataScene.transitionInserted.connect(self.transitionInserted)
        self.automataScene.stateNameChangedSignal.connect(self.stateNameChanged)
        self.automataScene.setActiveState(self.rootState)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.automataScene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)
        self.stateCanvas.setAcceptDrops(True)

    def stateInserted(self, state):
        if self.activeState != self.rootState:
            parent = self.treeModel.getByDataId(self.activeState.id)
            self.treeModel.insertState(state, QColor(Qt.white), parent)
        else:
            self.treeModel.insertState(state, QColor(Qt.white))

    def stateRemoved(self, state):
        if self.activeState != self.rootState:
            parent = self.treeModel.getByDataId(self.activeState.id)
            self.treeModel.removeState(state.stateData, parent)
        else:
            self.treeModel.removeState(state.stateData)

    def transitionInserted(self, tran):
        # print('transition inserted:' + tran.transitionData.name)
        pass

    def stateNameChanged(self, state):
        dataItem = self.treeModel.getByDataId(state.stateData.id)
        if dataItem != None:
            dataItem.name = state.stateData.name
            self.treeModel.layoutChanged.emit()

    def activeStateChanged(self):
        if self.automataScene.activeState != self.activeState:
            # print('visual states active state changed:' + self.automataScene.activeState.name)
            self.activeState = self.automataScene.activeState
            if self.activeState == self.rootState:
                self.treeView.selectionModel().clearSelection()
            else:
                self.treeView.setCurrentIndex(self.treeModel.indexOf(self.treeModel.getByDataId(self.activeState.id)))

    def upButtonClicked(self):
        if self.activeState != None:
            if self.activeState.parent != None:
                #print(self.activeState.parent.id)
                self.automataScene.setActiveState(self.activeState.parent)

    def getStateById(self,state, id):
        if state.id == id:
            return state
        else:
            result = None
            for child in state.getChildren():
                result = self.getStateById(child, id)
                if result is not None:
                    return result
            return result

    def treeItemClicked(self, index):
        # print('clicked item.id:' + str(index.internalPointer().id))
        state = self.getStateById(self.rootState, index.internalPointer().id)
        if state is not None:
            # set the active state as the loaded state
            self.automataScene.setActiveState(state)

    def timeStepDurationChanged(self, duration):
        if self.activeState is not None:
            self.activeState.setTimeStep(duration)

    def variablesChanged(self, variables):
        self.activeNamespace.setVariables(variables)

    def functionsChanged(self, functions):
        self.activeNamespace.setFuntions(functions)

    def librariesChanged(self, libraries):
        self.libraries = libraries

    def configChanged(self):
        if self.configDialog is not None:
            self.config = self.configDialog.getConfig()

    def getStateList(self, state, stateList):
        if len(state.getChildren()) > 0:
            stateList.append(state)

        for s in state.getChildren():
            self.getStateList(s, stateList)
