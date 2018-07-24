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
from gui.state.guistate import StateGraphicsItem
from core.transition import Transition
from xml.dom.minidom import Node

class State:
    def __init__(self, id, name, initial, namespace, parent=None):
        self.parent = parent
        self.id = id
        self.name = name
        self.code = ''
        self.timeStepDuration = 100
        self.x = 0
        self.y = 0
        self.initial = initial
        self.children = []
        self.originTransitions = []
        self.destTransitions = []
        
        self.namespace = namespace
        
        self.graphicsItem = None
        self.isRunning = False

    def setID(self, id):
        self.id = id

    def setNamespace(self, namespace):
        self.namespace = namespace

    def getNamespace(self):
        return self.namespace

    def getID(self):
        return self.id

    def setPos(self, x, y):
        self.x = x
        self.y = y

    def setParent(self, parent):
        self.parent = parent

    def addChild(self, child):
        if child not in self.children:
            self.children.append(child)

    def removeChild(self, child):
        if child in self.children:
            self.children.remove(child)

    def getChildren(self):
        return self.children

    def getOriginTransitions(self):
        return self.originTransitions

    def addOriginTransition(self, tran):
        if tran not in self.originTransitions:
            self.originTransitions.append(tran)

    def removeOriginTransition(self, tran):
        if tran in self.originTransitions:
            self.originTransitions.remove(tran)

    def getDestTransitions(self):
        return self.destTransitions

    def addDestTransition(self, tran):
        if tran not in self.destTransitions:
            self.destTransitions.append(tran)

    def removeDestTransition(self, tran):
        if tran in self.destTransitions:
            self.destTransitions.remove(tran)

    def getGraphicsItem(self):
        if self.graphicsItem == None:
            self.graphicsItem = StateGraphicsItem(self)
            self.graphicsItem.posChanged.connect(self.posChanged)
            self.graphicsItem.setRunning(self.isRunning)
        return self.graphicsItem

    def resetGraphicsItem(self):
        self.graphicsItem = None

    def posChanged(self, stateItem):
        scenePos = stateItem.scenePos()
        self.x = scenePos.x()
        self.y = scenePos.y()

    # creates a new copy of the state without children and transitions
    def getNewCopy(self):
        copy = State(self.id, self.name, False, self.parent)
        copy.code = self.code
        copy.x = self.x
        copy.y = self.y
        return copy

    def parseElement(self, elementName, parentElement):
        elements = parentElement.getElementsByTagName(elementName)
        if len(elements) > 0:
            if len(elements[0].childNodes) > 0:
                return elements[0].childNodes[0].nodeValue
        return ''

    def parse(self, stateElement):
        # parse attributes of the state
        for (name, value) in stateElement.attributes.items():
            if name == 'id':
                self.id = int(value)
            elif name == 'initial':
                self.initial = (value == 'True')

        self.name = stateElement.getElementsByTagName('name')[0].childNodes[0].nodeValue
        # print('read name:' + self.name)
        self.x = float(stateElement.getElementsByTagName('posx')[0].childNodes[0].nodeValue)
        self.y = float(stateElement.getElementsByTagName('posy')[0].childNodes[0].nodeValue)

        self.code = self.parseElement('code', stateElement)
        self.timeStepDuration = int((self.parseElement('timestep', stateElement)))

        # recursive child state parsing
        allChildTransitions = []
        statesById = {}
        namespaceNodes = []
        stateTransitions = []
        for childNode in stateElement.childNodes:
            if childNode.nodeType == Node.ELEMENT_NODE:
                if childNode.tagName == 'state':
                    childState = State(0, 'state', False, 0, self)
                    transitionNodes = childState.parse(childNode)
                    # print('add child:' + childState.name + ' to parent:' + self.name)
                    self.addChild(childState)
                    statesById[childState.id] = childState
                    allChildTransitions = allChildTransitions + transitionNodes
                elif childNode.tagName == 'transition':
                    stateTransitions.append(childNode)
                elif childNode.tagName == 'namespace':
                    self.namespaceid = childNode.stateElement.getAttribute('id')
                    namespaceNodes.append(childNode)

        # wire transitions with the states after all the child states are parsed
        for tranNode in allChildTransitions:
            transition = Transition(0, 'transition', 0)
            transition.parse(tranNode, statesById)

        for namespaceNode in namespaceNodes:
            namespace = Namespace(0, 'localnamespace', '', '')
            namespace.parse(childNode)

        # return transitions of the state to be able to wire after all states are created
        return stateTransitions, namespaceNodes

    def createElement(self, doc, parentElement=None):
        stateElement = doc.createElement('state')
        stateElement.setAttribute('initial', str(self.initial))
        stateElement.setAttribute('id', str(self.id))
        posxElement = doc.createElement('posx')
        posxElement.appendChild(doc.createTextNode(str(self.x)))
        posyElement = doc.createElement('posy')
        posyElement.appendChild(doc.createTextNode(str(self.y)))
        stateElement.appendChild(posxElement)
        stateElement.appendChild(posyElement)
        nameElement = doc.createElement('name')
        nameElement.appendChild(doc.createTextNode(self.name))
        stateElement.appendChild(nameElement)
        codeElement = doc.createElement('code')
        codeElement.appendChild(doc.createTextNode(self.code))
        stateElement.appendChild(codeElement)
        timeElement = doc.createElement('timestep')
        timeElement.appendChild(doc.createTextNode(str(self.timeStepDuration)))
        stateElement.appendChild(timeElement)

        # create transition elements
        for tran in self.getOriginTransitions():
            tranElement = tran.createElement(doc)
            stateElement.appendChild(tranElement)

        for child in self.getChildren():
            child.createElement(doc, stateElement)

        if parentElement is not None:
            parentElement.appendChild(stateElement)

        if self.initial and self.children:
            stateElement.appendChild(self.namespace.createNode(doc))
        return stateElement

    def getCode(self):
        return self.code

    def setCode(self, code):
        self.code = code

    def getTimeStep(self):
        return self.timeStepDuration

    def setTimeStep(self, timestep):
        self.timeStepDuration = timestep

    def getInitialChild(self):
        for child in self.getChildren():
            if child.initial:
                return child

        return None

    def setInitial(self, initial):
        self.initial = initial

    def getChildrenTransitions(self):
        childTransitions = []
        for child in self.getChildren():
            childTransitions += child.getOriginTransitions()
        return childTransitions

    def setRunning(self, status):
        self.isRunning = status
        if self.graphicsItem is not None:
            self.graphicsItem.setRunning(self.isRunning)

        if not self.isRunning:
            for child in self.getChildren():
                child.setRunning(self.isRunning)
