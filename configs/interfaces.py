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

import os
# a class to discover JdeRobot and ROS interfaces
from os import listdir
from os.path import isfile, join

class Interfaces:

    interfaces = None

    @staticmethod
    def getInterfaces(jderobot_install_prefix='/opt/jderobot/'):
        if Interfaces.interfaces is None:
            Interfaces.interfaces = {}
            interfaces_path = jderobot_install_prefix + '/include/jderobot/comm/interfaces/'
            from os import listdir
            from os.path import isfile, join
            if os.path.exists(interfaces_path):
                onlyfiles = [f for f in listdir(interfaces_path) if isfile(join(interfaces_path, f))]
                for interface_file in onlyfiles:
                    fp = open(interfaces_path+interface_file)
                    for line in fp:
                        class_index = line.find('class')
                        class_name_index = line.find('Client')
                        if class_index >= 0 and class_name_index >= 0 and class_index < class_name_index:
                            line[class_index:class_name_index].find(' ')
                            class_name = line[class_index+5:class_name_index+6].strip()
                            class_short_name = line[class_index+5:class_name_index].strip()
                            Interfaces.interfaces[class_short_name] = class_name
        return Interfaces.interfaces

    @staticmethod
    def getRosMessageTypes(rosDir = '/opt/ros/kinetic'):
        messageDir = rosDir + '/include'
        if os.path.exists(messageDir):
            allContents = os.listdir(messageDir)
            messages = []
            for entry in allContents:
                if os.path.isdir(messageDir + '/' + entry):
                    if entry.find('_msgs') >= 0:
                        messages.append(entry)

            types = []
            for msg in messages:
                typeDir = msg
                for entry in os.listdir(messageDir + '/' + msg):
                    if os.path.isfile(messageDir + '/' + msg + '/' + entry):
                        if entry.find('.h') >= 0 and entry[0].isupper():
                            type = {}
                            type['typeDir'] = typeDir
                            type['type'] = entry[:entry.find('.h')]
                            types.append(type)
            return types

        else:
            return False

if __name__ == '__main__':
    interfaces = Interfaces.getInterfaces()
    print(str(interfaces))

    #types = Interfaces.getRosMessageTypes()
    #for type in types:
    #    print(type['typeDir'] + '::' + type['type'])
