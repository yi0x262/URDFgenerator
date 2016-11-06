#!/usr/env/bin python3
from lxml import etree as et
from itertools import combinations_with_replacement
from copy import copy

def exBrackets(brackets):
    #list[1,2.0,'a'] -> str'1 2.0 a'
    string = ''
    for i in brackets:
        string += str(i)+' '
    return string[:-1]


class genURDF(object):
    zero = (0,0,0)
    def __init__(self,robotname):
        #robot init
        self.robot = et.Element('robot',name=robotname)
#frequency tags
    def origin(self,xyz,rpy):
        return et.Element('origin',xyz=exBrackets(xyz),rpy=exBrackets(rpy))

#for link
    #inertial
    def inertia(self,iner):
        #<inertia ixx="i[0]" .../>
        i = [str(n) for n in iner]
        return et.Element('inertia',ixx=i[0],ixy=i[1],ixz=i[2],iyy=i[3],iyz=i[4],izz=i[5])
    def inertia_box(self,mass,whd):
        #inertia wrapper
        inerxyz = []
        xyz = {0,1,2}
        for a in xyz:
            #ixx = 1/12 * m * (y*y + z*z)
            #iyy =            (x*x + z*z)
            #izz =            (x*x + y*y)
            #otherwise = 0
            iner = (1/12)*mass*sum([whd[i]**2 for i in (xyz-{a})])
            inerxyz.append(iner)
        return self.inertia((inerxyz[0],0,0,inerxyz[1],0,inerxyz[2]))
    def inertial(self,mass,inertia):
        #<inertial>
        ##<mass value="mass">
        ##<inertia/>
        #</inertial>
        inertial = et.Element('inertial')
        et.SubElement(inertial,'mass',value=str(mass))
        inertial.append(inertia)
        return inertial
    #geometry
    def geometry(self,shapegeom):
        ##<geometry>
        ###<shape/>
        ##</geometry>
        geometry = et.Element('geometry')
        geometry.append(shapegeom)
        return geometry
    def boxgeometry(self,whd):
        #<box size="w h d"/>
        return et.Element('box',size=exBrackets(whd))
    def cylindergeometry(self,lengrad):
        #<cylinder length="" radius=""/>
        return et.Element('cylinder',length=str(lengrad[0]),radius=str(lengrad[1]))
    #visual&collision
    def origin_geometry(self,linkname,tagtype,origin,geometry):
        ##<origin/>
        ##<geometry/>
        og = et.Element(tagtype,name=linkname)#+'_'+tagtype)
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
        gazebo = et.Element('gazebo',reference=refname)
        #color
        if not color is None:
            et.SubElement(gazebo,'material').text = 'Gazebo/'+color
        #self collide
        if not selfcollide is None:
            et.SubElement(gazebo,'selfCollide','true')
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
        noise = et.Element('noise')
        et.SubElement(noise,'type').text='gaussian'
        et.SubElement(noise,'mean').text='0.0'
        et.SubElement(noise,'stddev').text='0.03'
        return noise
    def sensor_imu_(self,bodyname):
        #<plugin name="boyname_imu" filename="libhector_gazebo_ros_imu.so">
        ##<updateRate>100.0</>
        ##<bodyName>
        plugin = et.Element('plugin',name=bodyname+'_imu',filename='libhector_gazebo_ros_imu.so')
        et.SubElement(plugin,'updateRate').text='100.0'
        et.SubElement(plugin,'bodyName').text=bodyname
        #et.SubElement(plugin,'frameId',)
        et.SubElement(plugin,'topicName').text='/{}/imu'.format(bodyname)
        et.SubElement(plugin,'rpyOffset').text='0 0 0'
        et.SubElement(plugin,'gaussianNoise').text='0'
        et.SubElement(plugin,'accelDrift').text='0.5 0.5 0.5'
        et.SubElement(plugin,'accelGaussianNoise').text='0.35 0.35 0.3'
        et.SubElement(plugin,'rateDrift').text='0.1 0.1 0.1'
        et.SubElement(plugin,'rateGaussianNoise').text='0.05 0.05 0.015'
        return plugin
    def sensor_imu__(self,bodyname):
        sensor = et.Element('sensor',name='imu_sensor',type='imu')
        et.SubElement(sensor,'visualize').text='1'
        et.SubElement(sensor,'update_rate').text='20'
        et.SubElement(sensor,'always_on').text='1'
        et.SubElement(sensor,'topic').text='__default_topic__'
        et.SubElement(sensor,'pose').text='0 0 0 0 0 0'
        imu = et.SubElement(sensor,'imu')
        imu.append(self.noise_gaussian())
        plugin = et.SubElement(sensor,'plugin',name='gazebo_ros_imu',filename='libhector_gazebo_ros_imu.so')
        et.SubElement(plugin,'topicName').text='/imu/'+bodyname
        et.SubElement(plugin,'frameName').text=bodyname
        et.SubElement(plugin,'updateRateHZ').text='10.0'
        et.SubElement(plugin,'gaussianNoise').text='0.0'
        et.SubElement(plugin,'xyzOffset').text='0,0,0'
        et.SubElement(plugin,'rpyOffset').text='0,0,0'
        return sensor
    def sensor_imu(self,bodyname):
        #controller = et.Element('controller_colon_gazebo_ros_imu',name='imu_controller',plugin='libgazebo_ros_imu')
        #controller = et.Element('sensor',type='imu',name='imu_controller',plugin='libgazebo_ros_imu.so')
        controller = et.Element('controller_colon_gazebo_ros_imu',name='imu_controller',plugin='libgazebo_ros_imu.so')
        et.SubElement(controller,'alwaysOn').text='true'
        et.SubElement(controller,'updateRate').text='100.0'
        et.SubElement(controller,'bodyName').text=bodyname
        #et.SubElement(controller,'link').text=bodyname
        et.SubElement(controller,'topicName').text='imu_data'
        et.SubElement(controller,'gaussianNoise').text='2.89e-08'
        et.SubElement(controller,'xyzOffsets').text='0 0 0'
        et.SubElement(controller,'rpyOffsets').text='0 0 0'
        et.SubElement(controller,'interface_colon_position',name='imu_position')
        #sensor = et.Element('sensor',name=linkname+'_imu',update_rate='50')
        return controller
    #link
    def link(self,name,origin,inertial,geometry,gazebo):
        #init link
        #<link>
        ##<inertial/>
        ##<visual/>
        ##<collision/>
        #</link>
        #<gazebo reference=name/>
        link = et.SubElement(self.robot,'link',name=name)
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
        parent = et.Element('parent',link=parentname)
        child = et.Element('child',link=childname)
        return parent,child
    #revolute
    def axis(self,axisxyz):
        return et.Element('axis',xyz=exBrackets(axisxyz))
    def limits_revolute(self,limits):
        #<limit effort="l[0]" lower="l[1]" upper.../>
        l = [str(limit) for limit in limits]
        return et.Element('limit',effort=l[0],lower=l[1],upper=l[2],velocity=l[3])
    #transmission
    def hardwareInterface(self,interface):#this is not smart. I recommend you to rewrite this.
        #<hardwareInterface>interface</hardwareInterface>
        hi = et.Element('hardwareInterface')
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

        tr = et.Element('transmission',name=jointname+'_tr')
        #type
        et.SubElement(tr,'type').text = 'transmission_interface/SimpleTransmission'
        #joint
        joint = et.SubElement(tr,'joint',name=jointname)
        joint.append(self.hardwareInterface('EffortJointInterface'))
        #actuator
        actuator = et.SubElement(tr,'actuator',name=jointname+'_mtr')
        actuator.append(self.hardwareInterface('EffortJointInterface'))
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
        joint = et.SubElement(self.robot,'joint',name=name,type='revolute')
        joint.extend(self.parentchild(parent,child))
        joint.append(self.origin(xyz,rpy))
        joint.append(self.axis(axis))
        joint.append(self.limits_revolute(limits))
        joint.append(self.transmission(name))
#output
    def gazebo_ros_control(self,robotname):
        gazebo = et.Element('gazebo')
        plugin = et.SubElement(gazebo,'plugin',name='gazebo_ros_control',filename='libgazebo_ros_control.so')
        et.SubElement(plugin,'robotNamespace').text = robotname
        et.SubElement(plugin,'robotSimType').text = 'gazebo_ros_control/DefaultRobotHWSim'
        return gazebo
    def print_urdf(self,robotname):
        self.robot.append(self.gazebo_ros_control(robotname))
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
    gu.link_box('link2',[0,0,1.5],[0,0,0],[1,1,1],10,'Orange',sensor='imu')
    gu.joint_revolute('joint1','link1','link2',[0,0,2],[0,0,0],[0,0,1],[10,-1.5,1.5,1])
    print(gu.print_urdf(robotname))
