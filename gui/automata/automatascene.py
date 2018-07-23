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
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsItem, QAction, QMenu
from PyQt5.QtCore import Qt, pyqtSignal
from gui.state.guistate import StateGraphicsItem
from gui.transition.guitransition import TransitionGraphicsItem
from gui.state.renamediaolog import RenameDialog
from gui.state.codedialog import CodeDialog
from gui.transition.transitioncodedialog import TransitionCodeDialog
from gui.transition.transitiontype import TransitionType
from gui.automata.optype import OpType
from core.state import State
from core.transition import Transition
from core.namespace import Namespace
from gui.state.idtextboxgraphicsitem import IdTextBoxGraphicsItem

class AutomataScene(QGraphicsScene):
    # signals
    stateInserted = pyqtSignal('QGraphicsItem')
    stateRemoved = pyqtSignal('QGraphicsItem')
    transitionInserted = pyqtSignal('QGraphicsItem')
    transitionRemoved = pyqtSignal('QGraphicsItem')
    stateNameChangedSignal = pyqtSignal('QGraphicsItem')
    activeStateChanged = pyqtSignal()
    activeNamespaceChanged = pyqtSignal()

    def __init__(self, parent=None):
        super(QGraphicsScene, self).__init__(parent)

        self.operationType = None
        self.operationData = None

        # transition origin and destination
        self.origin = None
        self.destination = None

        self.stateIndex = 0
        self.transitionIndex = 0
        self.namespaceIndex = 1

        self.prevOperationType = None
        self.stateTextEditingStarted = False

        # the active state whose children will be drawn on the graphicsview
        self.activeState = None
        self.activeNamespace = None

        self.createActions()

        self.selectedState = None
        self.selectedTransition = None
        self.contextPosition = None

        self.copiedState = None

    def createActions(self):

        # state actions
        self.renameStateAction = QAction('Rename', self)
        self.renameStateAction.triggered.connect(self.renameState)

        self.stateCodeAction = QAction('Code', self)
        self.stateCodeAction.triggered.connect(self.editStateCode)

        self.makeInitialAction = QAction('Make Initial', self)
        self.makeInitialAction.triggered.connect(self.makeInitial)

        self.copyStateAction = QAction('Copy', self)
        self.copyStateAction.triggered.connect(self.copyState)

        self.removeStateAction = QAction('Remove', self)
        self.removeStateAction.triggered.connect(self.removeState)

        self.pasteStateAction = QAction('Paste', self)
        self.pasteStateAction.triggered.connect(self.pasteState)

        # transition actions
        self.renameTransitionAction = QAction('Rename', self)
        self.renameTransitionAction.triggered.connect(self.renameTransition)

        self.transitionCodeAction = QAction('Code', self)
        self.transitionCodeAction.triggered.connect(self.editTransitionCode)

        self.removeTransitionAction = QAction('Remove', self)
        self.removeTransitionAction.triggered.connect(self.removeTransition)

    def renameState(self):
        dialog = RenameDialog('Rename', self.selectedState.stateData.name)
        dialog.move(self.contextPosition)
        dialog.nameChanged.connect(self.externalStateNameChanged)
        dialog.exec_()

    def editStateCode(self, state):
        dialog = CodeDialog('State Code', self.selectedState.stateData.code)
        dialog.move(self.contextPosition)
        dialog.codeChanged.connect(self.stateCodeChanged)
        dialog.exec_()

    def makeInitial(self):
        # there could be only one initial state
        for child in self.activeState.getChildren():
            child.setInitial(False)
            child.getGraphicsItem().setInitial(False)

        self.selectedState.setInitial(True)
        self.selectedState.stateData.setInitial(True)

    def copyState(self):
        self.copiedState = self.selectedState.stateData.getNewCopy()

    def pasteState(self):
        self.copiedState.id = self.getStateIndex()
        self.copiedState.x = self.currentScenePos.x()
        self.copiedState.y = self.currentScenePos.y()
        self.addStateItem(self.copiedState.getGraphicsItem())
        self.copyState()

    def removeState(self):
        self.removeStateItem(self.selectedState)

    def renameTransition(self):
        dialog = RenameDialog('Rename', self.selectedTransition.transitionData.name)
        dialog.move(self.contextPosition)
        dialog.nameChanged.connect(self.externalTransitionNameChanged)
        dialog.exec_()

    def editTransitionCode(self):
        dialog = TransitionCodeDialog('Transition Code', self.selectedTransition.transitionData)
        dialog.move(self.contextPosition)
        dialog.codeChanged.connect(self.transitionCodeChanged)
        dialog.exec_()

    def removeTransition(self):
        self.removeItem(self.selectedTransition)
        transition = self.selectedTransition.transitionData
        transition.origin.removeOriginTransition(transition)
        transition.destination.removeDestTransition(transition)

    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        QGraphicsScene.mousePressEvent(self, qGraphicsSceneMouseEvent)

    def addTransitionItem(self, tranItem, isInsertion=True):
        self.addItem(tranItem)
        if isInsertion:
            self.transitionInserted.emit(tranItem)

    def addStateItem(self, stateItem, isInsertion=True):
        stateItem.stateNameChanged.connect(self.stateNameChanged)
        stateItem.stateTextEditStarted.connect(self.stateTextEditStarted)
        stateItem.stateTextEditFinished.connect(self.stateTextEditFinished)

        self.addItem(stateItem)
        self.activeState.addChild(stateItem.stateData)
        if len(self.activeState.getChildren()) == 1:
            self.selectedState = stateItem
            self.makeInitial()

        if isInsertion:
            self.stateInserted.emit(stateItem)

    def removeStateItem(self, stateItem):
        stateItem.stateNameChanged.disconnect(self.stateNameChanged)
        stateItem.stateTextEditStarted.disconnect(self.stateTextEditStarted)
        stateItem.stateTextEditFinished.disconnect(self.stateTextEditFinished)

        for tran in stateItem.stateData.getOriginTransitions():
            print('removing origin tran:' + tran.name)
            tran.destination.removeDestTransition(tran)
            self.removeItem(tran.getGraphicsItem())
            self.transitionRemoved.emit(tran.getGraphicsItem())

        for tran in stateItem.stateData.getDestTransitions():
            print('removing destination tran:' + tran.name)
            tran.origin.removeOriginTransition(tran)
            self.removeItem(tran.getGraphicsItem())
            self.transitionRemoved.emit(tran.getGraphicsItem())

        if self.origin == stateItem:
            self.origin = None

        self.removeItem(stateItem)
        self.activeState.removeChild(stateItem.stateData)

        self.stateRemoved.emit(stateItem)

    """
    def importItems(self, stateItem):
        transitions = []
        for child in stateItem.getChildren():
            self.addStateItem(child.getGraphicsItem())
            transitions += child.getOriginTransitions()

        for tran in transitions:
            self.addTransitionItem(tran.getGraphicsItem(),False)
    """

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):
        # if we were editing the state text next mouse release should disable text editing
        # and should not add a new state or transition
        if self.stateTextEditingStarted:
            self.stateTextEditingStarted = False
            QGraphicsScene.mouseReleaseEvent(self, qGraphicsSceneMouseEvent)
            return

        if self.operationType == OpType.ADDSTATE and qGraphicsSceneMouseEvent.button() == Qt.LeftButton and self.operationData != None:
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) == 0:
                sIndex = self.getStateIndex()
                state = State(sIndex, 'state' + str(sIndex), False, self.operationData, self.activeState)
                state.setPos(qGraphicsSceneMouseEvent.scenePos().x(),
                             qGraphicsSceneMouseEvent.scenePos().y())
                self.addStateItem(state.getGraphicsItem())

            self.origin = None

        elif self.operationType == OpType.ADDTRANSITION and qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) > 0:
                # get the parent
                item = self.getParentItem(selectedItems[0])
                if isinstance(item, StateGraphicsItem):
                    if self.origin != None:
                        self.destination = item
                        tIndex = self.getTransitionIndex()
                        tran = Transition(tIndex, 'transition ' + str(tIndex), self.operationData,
                                          self.origin.stateData, self.destination.stateData)
                        self.addTransitionItem(tran.getGraphicsItem())
                        self.origin = None
                        self.operationData = None
                    else:
                        self.origin = item
                else:
                    self.origin = None
            else:
                self.origin = None

        # Feature to add? While clicking on the active state paste all the states

        elif self.operationType == OpType.IMPORTSTATE and qGraphicsSceneMouseEvent.button() == Qt.LeftButton and self.operationData != None:
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) == 0:
                self.importItems(self.operationData)
                self.setLastIndexes(self.activeState)
            self.operationData = None

        else:
            if self.operationType == OpType.OPENAUTOMATA:
                self.operationType = self.prevOperationType
                self.operationData = self.activeNamespace

        QGraphicsScene.mouseReleaseEvent(self, qGraphicsSceneMouseEvent)

    def contextMenuEvent(self, qGraphicsSceneContextMenuEvent):
        selectedItems = self.items(qGraphicsSceneContextMenuEvent.scenePos())
        if len(selectedItems) > 0:
            item = self.getParentItem(selectedItems[0])
            if isinstance(item, StateGraphicsItem):
                self.showStateContextMenu(item, qGraphicsSceneContextMenuEvent)
            elif isinstance(item, TransitionGraphicsItem):
                self.showTransitionContextMenu(item, qGraphicsSceneContextMenuEvent)
        elif len(selectedItems) == 0:
            self.showSceneContextMenu(qGraphicsSceneContextMenuEvent)

    def mouseDoubleClickEvent(self, qGraphicsSceneMouseEvent):
        selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
        if len(selectedItems) > 0:
            if isinstance(selectedItems[0], IdTextBoxGraphicsItem):
                QGraphicsScene.mouseDoubleClickEvent(self, qGraphicsSceneMouseEvent)
                pass
            else:
                item = self.getParentItem(selectedItems[0])
                if isinstance(item, StateGraphicsItem):
                    if item.stateData.getInitialChild():
                        self.setActiveNamespace(item.stateData.getInitialChild().getNamespace())
                    else:
                        nIndex = self.getNamespaceIndex()
                        namespace = Namespace(nIndex, 'namespace ' + str(nIndex), '', '')
                        self.setActiveNamespace(namespace)
                    self.setActiveState(item.stateData)
                QGraphicsScene.mouseDoubleClickEvent(self, qGraphicsSceneMouseEvent)

        self.prevOperationType = self.operationType
        self.operationType = OpType.OPENAUTOMATA

    def showStateContextMenu(self, stateItem, qEvent):
        cMenu = QMenu()
        cMenu.addAction(self.renameStateAction)
        cMenu.addAction(self.stateCodeAction)
        cMenu.addAction(self.makeInitialAction)
        cMenu.addAction(self.copyStateAction)
        cMenu.addAction(self.removeStateAction)
        self.selectedState = stateItem
        self.contextPosition = qEvent.screenPos()
        action = cMenu.exec_(qEvent.screenPos())

    def showTransitionContextMenu(self, tranItem, qEvent):
        cMenu = QMenu()
        cMenu.addAction(self.renameTransitionAction)
        cMenu.addAction(self.transitionCodeAction)
        cMenu.addAction(self.removeTransitionAction)
        self.selectedTransition = tranItem
        self.contextPosition = qEvent.screenPos()
        action = cMenu.exec_(qEvent.screenPos())

    def showSceneContextMenu(self, qEvent):
        cMenu = QMenu()
        cMenu.addAction(self.pasteStateAction)
        self.currentScenePos = qEvent.scenePos()
        action = cMenu.exec_(qEvent.screenPos())

    def setOperationType(self, type, data=None):
        self.operationType = type
        self.operationData = data

    def getStateIndex(self):
        self.stateIndex += 1
        return self.stateIndex

    def getTransitionIndex(self):
        self.transitionIndex += 1
        return self.transitionIndex

    def getNamespaceIndex(self):
        self.namespaceIndex += 1
        return self.namespaceIndex

    def getParentItem(self, item):
        while item.parentItem() is not None:
            item = item.parentItem()
        return item

    def externalStateNameChanged(self, newName):
        self.selectedState.textGraphics.setPlainText(newName)
        self.selectedState.textGraphics.textChanged.emit(newName)

    def externalTransitionNameChanged(self, newName):
        self.selectedTransition.transitionData.name = newName
        self.selectedTransition.textGraphics.setPlainText(newName)
        self.selectedTransition.textGraphics.textChanged.emit(newName)

    def stateCodeChanged(self, newCode):
        self.selectedState.stateData.code = newCode

    def transitionCodeChanged(self, type, typeValue, code):
        transition = self.selectedTransition.transitionData
        transition.setType(type)
        if type == TransitionType.TEMPORAL:
            transition.setTemporalTime(typeValue)
        elif type == TransitionType.CONDITIONAL:
            transition.setCondition(typeValue)

        transition.setCode(code)

    def stateNameChanged(self, state):
        self.stateNameChangedSignal.emit(state)

    def stateTextEditStarted(self):
        # temporarily disable operation type while editing the text
        self.prevOperationType = self.operationType
        self.operationType = None
        self.stateTextEditingStarted = True

    def stateTextEditFinished(self):
        # after text edit finishes restore operation type
        self.operationType = self.prevOperationType
        self.prevOperationType = None
        self.stateTextEditingStarted = True

    def setActiveState(self, state):
        if state != self.activeState:
            self.clearScene()
            self.activeState = state
            self.displayState(self.activeState)

    def setActiveNamespace(self, namespace):
        if namespace != self.activeNamespace:
            self.activeNamespace = namespace
            self.activeNamespaceChanged.emit()

    def displayState(self, state):
        transitions = []
        for child in state.getChildren():
            self.addStateItem(child.getGraphicsItem(), False)
            transitions = transitions + child.getOriginTransitions()

        for tran in transitions:
            self.addTransitionItem(tran.getGraphicsItem(), False)

        # print('set active state:' + self.activeState.name)
        self.activeStateChanged.emit()

    def clearScene(self):
        # clear scene
        self.clear()
        if self.activeState != None:
            # reset all of the graphics item of the current active state
            # print('type:' + str(type(self.activeState)))
            for child in self.activeState.getChildren():
                child.resetGraphicsItem()
                for tran in child.getOriginTransitions():
                    tran.resetGraphicsItem()

    def resetIndexes(self):
        """Reset Indices of Automata
        Helper Funtion for creating new AutomataScene"""
        self.stateIndex = 0
        self.transitionIndex = 0
        self.namespaceIndex = 0

    def setLastIndexes(self, rootState, rootNamespace):
        """Updates AutomataScene's Largest State and Transition ID"""
        if rootState.id > self.stateIndex:
            self.stateIndex = rootState.id

        if rootNamespace.id > self.namespaceIndex:
            self.namespaceIndex = rootNamespace.id

        for tran in rootState.getOriginTransitions():
            if tran.id > self.transitionIndex:
                self.transitionIndex = tran.id

        for child in rootState.getChildren():
            self.setLastIndexes(child)
