#!/usr/env/bin python3
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

        sub(self.launch,'node',od)#add launch file

    def controller_spawner(self,robotname,args):
        #self.node('controller_spawner','gazebo_ros',''{'args':args}
        pass

    def robot_state_publisher(self):
        self.node('robot_state_publisher','robot_state_publisher','state_publisher')

    def spawn_urdf(self,robotname,urdfpath):
        self.node('spawn_urdf','gazebo_ros','spawn_model',args='-file {} -urdf -model {}'.format(urdfpath,robotname))

    def world(self,**args):
        include = element('include',file='$(find gazebo_ros)/launch/empty_world.launch')
        for i,j in args.items():
            sub(include, 'arg', name=i, value=j)
        self.launch.append(include)

if __name__ == '__main__':
    c = genLAUNCH()
    c.world(                                    #http//:
            #world_name  = '',                  #
            paused      = 'false',              #start world stopped or not(default false)
            use_sim_time= 'true',               #gazebo_simulation_time syncronize ros_time or not(default true)
            gui         = 'true',               #
            headless    = 'false',              #
            debug       = 'false'               #
            )
    c.spawn_urdf('roboname','/home/yihome/')
    #c.robot_state_publisher()
    print(c)
