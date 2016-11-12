#!/usr/env/bin python3
from lxml import etree as et
from lxml.etree import Element as element
from lxml.etree import SubElement as sub
from itertools import combinations_with_replacement
from copy import copy

def exBrackets(brackets):
    #list -> str : [1,2.0,'a']->'1 2.0 a'
    string = ''
    for i in brackets:
        string += str(i)+' '
    return string[:-1]


class genURDF(object):
    zero = (0,0,0)
    def __init__(self,robotname):
        #robot init
        self.robot = element('robot',name=robotname)
        self.robot.append(self.gazebo_ros_control(robotname))#MUST write FIRST!!
#frequency tags
    def origin(self,xyz,rpy):
        return element('origin',xyz=exBrackets(xyz),rpy=exBrackets(rpy))

#for link
    #inertial
    def inertia(self,iner):
        #<inertia ixx="i[0]" .../>
        i = [str(n) for n in iner]
        return element('inertia',ixx=i[0],ixy=i[1],ixz=i[2],iyy=i[3],iyz=i[4],izz=i[5])
    def inertia_box(self,mass,whd):
        #inertia wrapper

        #ixx = 1/12 * m * (y*y + z*z)
        #iyy = 1/12 * m * (x*x + z*z)
        #izz = 1/12 * m * (x*x + y*y)
        #otherwise = 0

        inerxyz = []
        xyz = {0,1,2}
        for a in xyz:
            iner = (1/12)*mass*sum([whd[i]**2 for i in (xyz-{a})])
            inerxyz.append(iner)

        #inerxyz = [(1/12)*mass*sum([whd[i]**2 for i in (xyz-{a})]) for a in {0,1,2}]
        return self.inertia((inerxyz[0],0,0,inerxyz[1],0,inerxyz[2]))
    def inertial(self,mass,inertia):
        #<inertial>
        ##<mass value="mass">
        ##<inertia/>
        #</inertial>
        inertial = element('inertial')
        sub(inertial,'mass',value=str(mass))
        inertial.append(inertia)
        return inertial
    #geometry
    def geometry(self,shapegeom):
        ##<geometry>
        ###<shape/>
        ##</geometry>
        geometry = element('geometry')
        geometry.append(shapegeom)
        return geometry
    def boxgeometry(self,whd):
        #<box size="w h d"/>
        return element('box',size=exBrackets(whd))
    def cylindergeometry(self,lengrad):
        #<cylinder length="" radius=""/>
        return element('cylinder',length=str(lengrad[0]),radius=str(lengrad[1]))
    #visual&collision
    def origin_geometry(self,linkname,tagtype,origin,geometry):
        ##<origin/>
        ##<geometry/>
        og = element(tagtype,name=linkname)#+'_'+tagtype)
        og.append(origin)
        og.append(geometry)
        return og
    def visual(self,linkname,origin,geometry):
        ##<visual>
        ###<geometry/>
        ##</visual>
        return self.origin_geometry(linkname+'_visual','visual',origin,geometry)
    def collision(self,linkname,origin,geometry):
        ##<collision>
        ###<geometry/>
        ##</collision>
        return self.origin_geometry(linkname+'_collision','collision',origin,geometry)
    #gazebo reference
    def gazebo(self,refname,color=None,selfcollide=None,sensor=None):
        #<gazebo reference="refname">
        ##<material></material>
        ##<selfCollide></self...>
        ##<plugin/>
        gazebo = element('gazebo',reference=refname)
        #color
        if not color is None:
            sub(gazebo,'material').text = 'Gazebo/'+color
        #self collide
        if not selfcollide is None:
            sub(gazebo,'selfCollide','true')
        #sensor
        #!!!now printing...!!!#
        if sensor is None:
            return gazebo
        if sensor == 'imu':
            gazebo.append(self.sensor_imu(refname))

        return gazebo

    def noise_gaussian(self):
        #<noise>
        ##<type>gaussian</>
        ##<mean>0.0</>
        ##>stddev>0.03</>
        #</noise>
        noise = element('noise')
        sub(noise,'type').text='gaussian'
        sub(noise,'mean').text='0.0'
        sub(noise,'stddev').text='0.03'
        return noise
    def sensor_imu(self,bodyname):
        sensor = element('sensor',name='imu_'+bodyname,type='imu')

        plugin = sub(sensor,'plugin',name='gazebo_ros_imu',filename='libgazebo_ros_imu.so')
        sub(plugin,'topicName').text='/imu/'+bodyname
        sub(plugin,'bodyName').text=bodyname
        sub(plugin,'updateRate').text='10.0'
        sub(plugin,'gaussianNoise').text='0.0'
        sub(plugin,'xyzOffset').text='0,0,0'
        sub(plugin,'rpyOffset').text='0,0,0'
        return sensor
    #link
    def link(self,name,origin,inertial,geometry,gazebo):
        #init link
        #<link>
        ##<inertial/>
        ##<visual/>
        ##<collision/>
        #</link>
        #<gazebo reference=name/>
        link = sub(self.robot,'link',name=name)
        link.append(inertial)
        link.append(self.visual(name,copy(origin),copy(geometry)))
        link.append(self.collision(name,copy(origin),copy(geometry)))
        #gazebo reference
        self.robot.append(gazebo)
    def link_box(self,name,xyz,rpy,whd,mass,color=None,selfcollide=None,sensor=None):
        self.link(name,self.origin(xyz,rpy),
                self.inertial(mass,self.inertia_box(mass,whd)),
                self.geometry(self.boxgeometry(whd)),
                self.gazebo(name,color=color,selfcollide=selfcollide,sensor=sensor))
#for joint
    #common
    def parentchild(self,parentname,childname):
        parent = element('parent',link=parentname)
        child = element('child',link=childname)
        return parent,child
    #revolute
    def axis(self,axisxyz):
        return element('axis',xyz=exBrackets(axisxyz))
    def limits_revolute(self,limits):
        #<limit effort="l[0]" lower="l[1]" upper.../>
        l = [str(limit) for limit in limits]
        return element('limit',effort=l[0],lower=l[1],upper=l[2],velocity=l[3])
    #transmission
    def hardwareInterface(self,interface):#this is not smart. I recommend you to rewrite this.
        #<hardwareInterface>interface</hardwareInterface>
        hi = element('hardwareInterface')
        hi.text = interface
        return hi
    def transmission(self,jointname):
        #<transmission name="..._tr">
        ##<type></type>
        ##<joint>
        ###<hardwareInterface/>
        ##</joint>
        ##<actuator>
        ###<hardwareInterface/>
        ##</actuator>
        #</transmission>

        tr = element('transmission',name=jointname+'_tr')
        #type
        sub(tr,'type').text = 'transmission_interface/SimpleTransmission'
        #joint
        joint = sub(tr,'joint',name=jointname)
        joint.append(self.hardwareInterface('EffortJointInterface'))
        #actuator
        actuator = sub(tr,'actuator',name=jointname+'_mtr')
        actuator.append(self.hardwareInterface('EffortJointInterface'))
        sub(actuator,'mechanicalReduction').text = '1'
        return tr
    def joint_revolute(self,name,parent,child,xyz,rpy,axis,limits):
        #<joint name="name" type="revolute">
        ##<parent link="parentname"/>
        ##<child link="childname"/>
        ##<origin/>
        ##<axis xyz="xyz"/>
        ##<limit params="limits"/>#<-inaccuracy
        #</joint>
        #<transmission/>
        joint = sub(self.robot,'joint',name=name,type='revolute')
        joint.extend(self.parentchild(parent,child))
        joint.append(self.origin(xyz,rpy))
        joint.append(self.axis(axis))
        joint.append(self.limits_revolute(limits))
        self.robot.append(self.transmission(name))

#output
    def gazebo_ros_control(self,robotname):
        gazebo = element('gazebo')
        plugin = sub(gazebo,'plugin',name='gazebo_ros_control',filename='libgazebo_ros_control.so')
        sub(plugin,'robotNamespace').text = robotname
        sub(plugin,'robotSimType').text = 'gazebo_ros_control/DefaultRobotHWSim'
        return gazebo
    def __str__(self):
        xmldec = '<?xml version="1.0"?>\n'
        urdf = et.tostring(self.robot,pretty_print=True,xml_declaration=False).decode('utf-8').replace('_colon_',':')
        return xmldec + urdf

class genSimulation(object):
    def __init__(self,robotname):
        self.urdf = genURDF(robotname)

if __name__ == '__main__':
    robotname = 'test'
    gu = genURDF(robotname)
    gu.link_box('link1',[0,0,1],[0,0,0],[1,1,1],10,'White')
    gu.link_box('link2',[0,0,1.5],[0,0,0],[1,1,1],10,'Orange')#,sensor='imu')
    gu.joint_revolute('joint1','link1','link2',[0,0,2],[0,0,0],[0,0,1],[10,-1.5,1.5,1])
    print(gu.print_urdf(robotname))
