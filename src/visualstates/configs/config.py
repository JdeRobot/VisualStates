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

ROS = 1

class Config():
    def __init__(self):
        self.type = ROS

class RosConfig(Config):

    PUBLISH = 0
    SUBSCRIBE = 1

    def __init__(self):
        self.type = ROS
        self.topics = []
        self.buildDependencies = []
        self.runDependencies = []

    def updateROSConfig(self, newConfig):
        self.updateTopics(newConfig.topics)
        self.updateBuildDependencies(newConfig.buildDependencies)
        self.updateRunDependencies(newConfig.runDependencies)

    def getTopics(self):
        return self.topics

    def isTopicByName(self, topicName):
        topicNames = map(lambda x: x['name'], self.topics)
        if topicName in topicNames:
            return False
        else:
            return True

    def addTopic(self, id, name, type, opType):
        topic = {}
        topic['id'] = id
        topic['name'] = name
        topic['type'] = type
        topic['opType'] = opType
        self.topics.append(topic)

    def updateTopics(self, topics):
        for topic in topics:
            if topic not in self.topics:
                self.addTopic(self.getTopicID(), topic['name'], topic['type'], topic['opType'])

    def getTopicID(self):
        if self.topics:
            return max(map(lambda x: x['id'], self.topics)) + 1
        else: return 0

    def removeTopic(self, id):
        topicToDelete = None
        for t in self.topics:
            if t['id'] == id:
                topicToDelete = t
                break
        self.topics.remove(topicToDelete)

    def editTopic(self, id, name):
        for topic in self.topics:
            if topic['id'] == id:
                topic['name'] = name
                break

    def setBuildDependencies(self, dependencies):
        """Dependencies coming as strings"""
        self.buildDependencies = []
        dependStrs = dependencies.split('\n')
        for dStr in dependStrs:
            if len(dStr.strip()) > 0:
                self.buildDependencies.append(dStr.strip())

    def getBuildDependencies(self):
        return self.buildDependencies

    def getBuildDependenciesAsText(self):
        myStr = ''
        for i in range(len(self.buildDependencies)):
            myStr += self.buildDependencies[i]
            if i < len(self.buildDependencies):
                myStr += '\n'
        return myStr

    def updateBuildDependencies(self, dependencies):
        """Dependencies coming as List"""
        for dep in dependencies:
            if dep not in self.buildDependencies:
                self.buildDependencies.append(dep)

    def setRunDependencies(self, dependencies):
        """Dependencies coming as strings"""
        self.runDependencies = []
        dependStrs = dependencies.split('\n')
        for dStr in dependStrs:
            if len(dStr.strip()) > 0:
                self.runDependencies.append(dStr.strip())

    def getRunDependencies(self):
        return self.runDependencies

    def getRunDependenciesAsText(self):
        myStr = ''
        for i in range(len(self.runDependencies)):
            myStr += self.runDependencies[i]
            if i < len(self.runDependencies):
                myStr += '\n'
        return myStr

    def updateRunDependencies(self, dependencies):
        """Dependencies coming as List"""
        for dep in dependencies:
            if dep not in self.runDependencies:
                self.runDependencies.append(dep)

    def createNode(self, doc):
        cfgElement = doc.createElement('config')
        cfgElement.setAttribute('type', str(self.type))

        bDependencies = doc.createElement('buildDependencies')
        for bDepend in self.buildDependencies:
            dependElement = doc.createElement('dependency')
            dependElement.appendChild(doc.createTextNode(bDepend))
            bDependencies.appendChild(dependElement)
        cfgElement.appendChild(bDependencies)

        rDependencies = doc.createElement('runDependencies')
        for rDepend in self.runDependencies:
            dElement = doc.createElement('dependency')
            dElement.appendChild(doc.createTextNode(rDepend))
            rDependencies.appendChild(dElement)
        cfgElement.appendChild(rDependencies)

        tElements = doc.createElement('topics')
        for t in self.topics:
            tElement = doc.createElement('topic')
            tElement.setAttribute('id', str(t['id']))
            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(t['name']))
            tElement.appendChild(nameElement)
            typeElement = doc.createElement('type')
            typeElement.appendChild(doc.createTextNode(t['type']))
            tElement.appendChild(typeElement)
            opElement = doc.createElement('opType')
            opElement.appendChild(doc.createTextNode(t['opType']))
            tElement.appendChild(opElement)

            tElements.appendChild(tElement)

        cfgElement.appendChild(tElements)

        return cfgElement

    def loadNode(self, node):
        self.type = node.getAttribute('type')

        self.buildDependencies = []
        bDependencies = node.getElementsByTagName('buildDependencies')[0]
        for bDependency in bDependencies.getElementsByTagName('dependency'):
            self.buildDependencies.append(bDependency.childNodes[0].nodeValue)
            # print('bdepend:' + bDependency.childNodes[0].nodeValue)

        self.runDependencies = []
        rDependencies = node.getElementsByTagName('runDependencies')[0]
        for rDependency in rDependencies.getElementsByTagName('dependency'):
            self.runDependencies.append(rDependency.childNodes[0].nodeValue)
            # print('rdepend:' + rDependency.childNodes[0].nodeValue)

        self.topics = []
        tElements = node.getElementsByTagName('topics')[0]
        for t in tElements.getElementsByTagName('topic'):
            topic = {}
            topic['id'] = int(t.getAttribute('id'))
            topic['name'] = t.getElementsByTagName('name')[0].childNodes[0].nodeValue
            topic['type'] = t.getElementsByTagName('type')[0].childNodes[0].nodeValue
            topic['opType'] = t.getElementsByTagName('opType')[0].childNodes[0].nodeValue
            self.topics.append(topic)
