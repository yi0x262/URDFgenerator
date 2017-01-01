#!/usr/env/bin python3

#.launch file reference:https://sites.google.com/site/robotlabo/time-tracker/ros/gazebo_mani

from collections import OrderedDict

import lxml.etree as et
from lxml.etree import Element as element
from lxml.etree import SubElement as sub

class genLAUNCH(object):
    def __init__(self):
        self.launch = element('launch')

    def __str__(self):
        xmldec = '<?xml version="1.0"?>\n'
        launch = et.tostring(self.launch,pretty_print=True,xml_declaration=False).decode('utf-8')
        return xmldec+launch

    def node(self,nodename,pkgname,typename,**args):
        od = OrderedDict()
        od['name']  = nodename  #program name as a node on ros
        od['pkg']   = pkgname   #included package name  (like family name)
        od['type']  = typename  #truth name the program (like last name)
        for i,j in args.items():
            od[i]   = j         #additional arguments

        return element('node',od)#add launch file

    def controller_spawner(self,robotname,args):
        self.launch.append(self.node('controller_spawner','controller_manager','spawner',respawn='false',output='screen',ns='/'+robotname,args=args))


    def spawn_urdf(self,robotname,urdfpath):
        self.launch.append(self.node('spawn_urdf','gazebo_ros','spawn_model',args='-file {} -urdf -model {}'.format(urdfpath,robotname)))

    def empty_world(self,**args):
        include = element('include',file='$(find gazebo_ros)/launch/empty_world.launch')
        for i,j in args.items():
            sub(include, 'arg', name=i, value=j)
        self.launch.append(include)
    def robot_state_publisher(self,robotname,pkgname):
        sub(self.launch,'arg',name='model',default='$(find {})/urdf/{}.urdf'.format(pkgname,robotname))
        sub(self.launch,'param',name='robot_description',textfile='$(arg model)')
        sub(self.launch,'rosparam',file='$(find {})/config/{}.yaml'.format(pkgname,robotname),command='load')#very important

        jsp = self.node('joint_state_publisher','joint_state_publisher','joint_state_publisher')
        rsp = self.node('robot_state_publisher','robot_state_publisher','robot_state_publisher',respawn='false',output='screen')
        od = OrderedDict()#Because element(and python's func) cannot take 'from'(like import,def and so on) as an argument.
        od['from']  = '/joint_states'
        od['to']    = '/'+robotname+'/joint_states'
        rsp.append(element('remap',od))
        self.launch.append(rsp)

if __name__ == '__main__':
    robotname='ROBO'

    c = genLAUNCH()
    c.world(                                    #http//:
            #world_name  = '',                  #
            paused      = 'false',              #start world stopped or not(default false)
            use_sim_time= 'true',               #gazebo_simulation_time syncronize ros_time or not(default true)
            gui         = 'true',               #
            headless    = 'false',              #
            debug       = 'false'               #
            )
    import os
    c.spawn_urdf(robotname,os.getcwd())
    c.controller_spawner(robotname,'args args2 joint_state_controller')
    c.robot_state_publisher(robotname)
    print(c)
