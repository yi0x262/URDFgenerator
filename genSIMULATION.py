#!/usr/env/bin python3

"""
catkin_ws/src
    /my_simulation          #generate_path
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
    def __init__(self,robotname,path,pid=None,publish_rate=100):
        self.robotname = robotname#?
        self.urdf = genURDF(robotname)
        self.yaml = genYAML(robotname)
        self.launch = genLAUNCH()

        self.controller_names = str()
        self.generate_path = path
        if pid is not None:
            self.pid = pid

        self.controller_names += self.yaml.joint_state_controller(publish_rate)+' '


    def generate(self,dirname,extension,string):
        #urdf
        abspath = self.generate_path+'/'+dirname
        try:
            os.mkdir(abspath)#run after catkin_create_package
        except FileExistsError:
            pass
        f = open(abspath+'/'+self.robotname+'.'+extension,'w')
        f.write(string)
        f.close()
    def output(self,pkgname):
        self.controller_names = self.controller_names[:-1]# cut last space
        self.make_launch(pkgname)

        self.generate('urdf','urdf',str(self.urdf))
        self.generate('config','yaml',str(self.yaml))
        self.generate('launch','launch',str(self.launch))

    def link_box(self,*args,**keys):
    #def link_box(self,name,xyz,rpy,whd,mass,color=None,selfcollide=None,sensor=None):
        #print(args,keys)
        self.urdf.link_box(*args,**keys)
    def joint_revolute(self,name,*args,pid=(1,0,0)):
    #def joint_revolute(self,name,parent,child,xyz,rpy,axis,limits):
        _pid = self.pid if hasattr(self,'pid') else pid
        self.urdf.joint_revolute(name,*args)
        self.controller_names += self.yaml.revolute_joint_controller(name,pid=_pid)+' '
    def joint_fixed(self,*args):
        self.urdf.joint_fixed(*args)

    def make_launch(self,pkgname):
        self.launch.empty_world(                      #http//:
            #world_name  = '',                  #
            paused      = 'true',              #start world stopped or not(default false)
            use_sim_time= 'true',               #gazebo_simulation_time syncronize ros_time or not(default true)
            gui         = 'true',               #
            headless    = 'false',              #
            debug       = 'false'               #
            )
        print(self.generate_path+'/urdf/'+self.robotname+'.urdf')
        self.launch.controller_spawner(self.robotname,self.controller_names)
        self.launch.robot_state_publisher(self.robotname,pkgname)
        self.launch.spawn_urdf(self.robotname,self.generate_path+'/urdf/'+self.robotname+'.urdf')

if __name__ == '__main__':
    robotname = 'test'
    path = '/home/yihome/catkin_ws/src/test_gazebo'
    gs = genSIMULATION(robotname,path)
    gs.link_box('link1',[0,0,1],[0,0,0],[1,1,1],10,'White')
    gs.link_box('link2',[0,0,1.5],[0,0,0],[1,1,1],10,'Orange',sensor='imu')
    gs.joint_revolute('joint1','link1','link2',[0,0,2],[0,0,0],[0,0,1],[10,-1.5,1.5,1])

    gs.output()
