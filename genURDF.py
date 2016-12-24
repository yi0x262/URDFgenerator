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

        if not color is None:
            sub(gazebo,'material').text = 'Gazebo/'+color
        if not selfcollide is None:
            sub(gazebo,'selfCollide','true')
        #!!!now printing...!!!#
        if sensor == 'imu':
            self.sensor_imu(refname)

        return gazebo

    def mean_stddev_bias(self,name):
        parent = element(name)
        sub(parent,'mean').text='0'
        sub(parent,'stddev').text='0.0002'
        sub(parent,'bias_mean').text='7.5e-06'
        sub(parent,'bias_stddev').text='8e-07'
        return parent

    def noise_gaussian(self):
        #<noise>
        ##<type>gaussian</>
        ##<rate>
        ###<mean>0.0</>
        ###<stddev>0.03</>
        ###<bias_mean>7.5e-06</>
        ###<bias_stddev>8e-07</>
        ##</rate>
        ##<accel>
        ###...
        ##</accel>
        #</noise>
        noise = element('noise')
        sub(noise,'type').text='gaussian'
        rate = self.mean_stddev_bias('rate')
        accel = self.mean_stddev_bias('accel')
        noise.extend((rate,accel))
        return noise

    def gazebo_ros_imu(self,bodyname):
        #ref:https://gitlab.com/nasa-jsc-robotics/val_description/blob/ddaf8378741839f26ae6d734dbe6e4441b9b1e23/model/robots/common/xacro/sensors/microstrain.xacro
        gazebo = element('gazebo')
        plugin = element('plugin',name='gazebo_ros_imu',filename='libgazebo_ros_imu.so')
        sub(plugin,'topicName').text=bodyname+'/imu'
        sub(plugin,'bodyName').text=bodyname
        sub(plugin,'serviceName').text=bodyname+'/imu_service'
        sub(plugin,'frameName').text=bodyname+'_imuframe'
        sub(plugin,'updateRate').text='10.0'
        sub(plugin,'gaussianNoise').text='0.0'
        #sub(plugin,'xyzOffset').text='0,0,0'
        #sub(plugin,'rpyOffset').text='0,0,0'
        gazebo.append(plugin)
        return gazebo

    def sensor_imu(self,bodyname):
        self.robot.append(self.gazebo_ros_imu(bodyname))
        gazebo = element('gazebo',reference=bodyname+'_imuframe')
        sensor = element('sensor',name=bodyname+'_imu',type='imu')
        sub(sensor,'imuTransform').text='0 0 0 0 0 0'
        sub(sensor,'always_on').text='true'
        sub(sensor,'pose').text='0 0 0 0 0 0'
        imu = sub(sensor,'imu')
        imu.append(self.noise_gaussian())
        self.robot.append(gazebo)
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
