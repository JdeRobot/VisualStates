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

class Parameter:
    def __init__(self, name='', type='', value='', desc=''):
        self.name = name
        self.type = type
        self.value = value
        self.desc = desc

    def setName(self, name):
        self.name = name

    def setType(self, type):
        self.type = type

    def setValue(self, value):
        self.value = value

    def setDesc(self, desc):
        self.desc = desc

    def createDocFromParam(self, doc):
        paramElement = doc.createElement('param')
        paramElement.setAttribute('type', self.type)
        paramElement.setAttribute('name', self.name)

        valueElement = doc.createElement('value')
        valueElement.appendChild(doc.createTextNode(self.value))
        paramElement.appendChild(valueElement)
        descElement = doc.createElement('description')
        descElement.appendChild(doc.createTextNode(self.desc))
        paramElement.appendChild(descElement)

        return paramElement

    def parseElement(self, element):
        for (name, value) in element.attributes.items():
            if name == 'name':
                self.name = str(value)
            elif name == 'type':
                self.type = str(value)

        if len(element.getElementsByTagName('value')[0].childNodes) > 0:
            self.value = str(element.getElementsByTagName('value')[0].childNodes[0].nodeValue)
        else:
            self.value = ''
        if len(element.getElementsByTagName('description')[0].childNodes) > 0:
            self.desc = str(element.getElementsByTagName('description')[0].childNodes[0].nodeValue)
        else:
            self.desc = ''

def isTypeEqualValue(type, value):
    if type == 'Boolean' and not (value == 'True' or value == 'False' or value == 'true' or value == 'false'):
        return False
    elif type == 'Integer' and not value.isdigit():
        return False
    elif type == 'Float' and not (value.replace('.', '', 1).isdigit()):
        return False
    elif type == 'Character' and not len(value) == 1:
        return False
    else:
        return True

def isParamName(name):
    if name == '':
        return False
    elif name.replace('_', '').isalnum() and name[0].isalpha():
        return True
    else:
        return False
