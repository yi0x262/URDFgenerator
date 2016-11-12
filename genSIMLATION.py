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
    def __init__(self,robotname,path):
        self.robotname = robotname#?
        self.urdf = genURDF(robotname)
        self.yaml = genYAML(robotname)
        self.launch = genLAUNCH()

        self.controller_names = str()
        self.generate_path = path

    def generate(self,dirname,extension,string):
        #urdf
        abspath = self.path+'/'+dirname
        os.makedirs(abspath)
        f = open(abspath+'/'+self.robotname+'.'+extension)
        f.write(string)
        f.close()
    def output(self):
        self.controller_names += self.yaml.joint_state_controller(50)
        self.make_launch()

        self.generate('urdf','urdf',str(self.urdf))
        self.generate('config','yaml',str(self.yaml))
        self.generate('launch','launch',str(self.launch))

    def link_box(self,name,xyz,rpy,whd,mass,color=None,selfcollide=None,sensor=None):
        self.urdf.link_box(name,xyz,rpy,whd,mass,color,selfcollide,sensor)
    #def link_box(*args,**keyargs):
        #self.urdf.link_box(*args,**keyargs)
    def joint_revolute(self,name,parent,child,xyz,rpy,axis,limits):
        self.urdf.joint_revolute(name,parent,child,xyz,rpy,axis,limits)

        self.controller_names += self.yaml.revolute_joint_controller(name)+' '

    def make_launch(self):
        self.launch.


if __name__ == '__main__':
