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
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtWidgets import QMainWindow, QDockWidget, QTreeView, QGraphicsView, \
    QWidget, QLabel, QVBoxLayout, QPushButton, QGraphicsItem, \
    QGraphicsScene
from visualstates.gui.tree.treemodel import TreeModel
from visualstates.core.state import State
from visualstates.core.transition import Transition
from visualstates.configs.package_path import get_package_path

class RunTimeGui(QMainWindow):

    activeStateChanged = pyqtSignal(int)
    runningStateChanged = pyqtSignal(int)
    loadFromRoot = pyqtSignal(int)

    def __init__(self, parent=None):
        super(QMainWindow, self).__init__()

        self.setWindowTitle("VisualStates RunTime GUI")

        self.rootState = None

        # create status bar
        self.statusBar()

        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.states = {}
        self.transitions = {}

        self.activeState = None

        self.activeStateChanged.connect(self.activeStateChangedHandle)
        self.runningStateChanged.connect(self.runningStateChangedHandle)
        self.loadFromRoot.connect(self.loadFromRootHandle)

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
        logoPixmap = QPixmap(get_package_path() + '/share/resources/jderobot.png')
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
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 2000, 2000)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.scene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)

    def addState(self, id, name, initial, x, y, parentId):
        if parentId is not None:
            self.states[id] = State(id, name, initial, self.states[parentId])
            self.states[parentId].addChild(self.states[id])
            self.states[id].parent = self.states[parentId]
            parentItem = self.treeModel.getByDataId(parentId)
            # print('parent:' + str(parentItem))
        else:
            self.states[id] = State(id, name, initial, None)
        if id == 0:
            self.rootState = self.states[id]

        self.states[id].setPos(x, y)

    def addTransition(self, id, name, originId, destinationId, x, y):
        self.transitions[id] = Transition(id, name, self.states[originId], self.states[destinationId])
        self.transitions[id].setPos(x, y)

    def emitRunningStateById(self, id):
        self.runningStateChanged.emit(id)

    def runningStateChangedHandle(self, id):
        if id not in self.states:
            return

        runningState = self.states[id]
        runningState.setRunning(True)

        if runningState.parent is not None:
            for child in runningState.parent.getChildren():
                if child.getRunning():
                    child.setRunning(False)
                    self.scene.removeItem(child.getGraphicsItem())
                    child.resetGraphicsItem()
                    self.scene.addItem(child.getGraphicsItem())


            if not runningState.getRunning():
                runningState.setRunning(True)
                self.scene.removeItem(runningState.getGraphicsItem())
                runningState.resetGraphicsItem()
                self.scene.addItem(runningState.getGraphicsItem())

            self.treeModel.setAllBackgroundByParentId(Qt.white, runningState.parent.id)

        self.treeModel.setBackgroundById(runningState.id, Qt.green)

    def emitActiveStateById(self, id):
        self.activeStateChanged.emit(id)

    def activeStateChangedHandle(self, id):
        if self.activeState is not None:
            for child in self.activeState.getChildren():
                child.resetGraphicsItem()
                for tran in child.getOriginTransitions():
                    tran.resetGraphicsItem()

        self.activeState = self.states[id]
        # print('set active state:' + str(id))
        self.scene.clear()
        for childState in self.activeState.getChildren():
            qitem = childState.getGraphicsItem()
            qitem.setAcceptHoverEvents(False)
            qitem.setFlag(QGraphicsItem.ItemIsMovable, False)
            qitem.doubleClicked.connect(self.stateDoubleClicked)
            self.setAcceptDrops(False)
            self.scene.addItem(qitem)
            for tran in childState.getOriginTransitions():
                # print('add transition:' + str(tran.id))
                qitem = tran.getGraphicsItem()
                qitem.disableInteraction()
                self.scene.addItem(qitem)

    def emitLoadFromRoot(self):
        self.loadFromRoot.emit(0)

    def loadFromRootHandle(self, id):
        self.treeModel.loadFromRoot(self.states[id])
        self.states[id].setRunning(True)
        for child in self.states[id].getChildren():
            if child.initial:
                child.setRunning(True)
                self.treeModel.setBackgroundById(child.id, Qt.green)
                return

    def stateDoubleClicked(self, stateItem):
        if len(stateItem.stateData.getChildren()) > 0:
            self.emitActiveStateById(stateItem.stateData.id)

    def upButtonClicked(self):
        if self.activeState is not None:
            if self.activeState.parent is not None:
                self.emitActiveStateById(self.activeState.parent.id)

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
        pass

    def getStateList(self, state, stateList):
        if len(state.getChildren()) > 0:
            stateList.append(state)

        for s in state.getChildren():
            self.getStateList(s, stateList)