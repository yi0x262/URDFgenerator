#!/usr/env/bin python3

"""
catkin_ws/src
    /my_simulation
        package.xml
        CmakeLists.txt
        /urdf
            ROBOT.urdf      #generate by genURDF
        /launch
            ROBOT.launch    #generate by genLAUNCH
        /config
            ROBOT.yaml      #generate by genYAML
"""

import os

from genURDF import genURDF
from genYAML import genYAML
from genLAUNCH import genLAUNCH

class genSIMULATION(object):
    def __init__(self,robotname):
        self.robotname = robotname#?
        self.urdf = genURDF(robotname)
        self.yaml = genYAML(robotname)
        self.launch = genLAUNCH()

    def make_launch(self):
        pass

    def generate(self,dirname,extension,string):
        #urdf
        os.mkdir(dirname)
        f = open(dirname+'/'+self.robotname+'.'+extension)
        f.write(string)
        f.close()
    def output(self):
        self.generate('urdf','urdf',str(self.urdf))
        self.generate('config','yaml',str(self.yaml))
        self.generate('launch','launch',str(self.launch))
