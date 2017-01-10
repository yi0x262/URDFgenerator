#!/usr/env/bin python3
from genSIMULATION import genSIMULATION

import os
gs = genSIMULATION('khr3',os.getcwd(),pid=(1,0.01,0.1))

def arm(name,xyz,whd,mass,color,*options,rpy=(0,0,0)):
    keys = dict()
    #print(options)
    opts = set(options)
    #selfcollide
    if 'selfcollide' in opts:
        keys['selfcolide'] = True
    if 'imu' in opts:
        keys['sensors'] = 'imu'
    return name,xyz,rpy,whd,mass,color,keys

def rl(p,RorL):
    return [RorL*p[0],*p[1:]]
def changeRorL(profile,RorL):
    return [rl(profile[0],RorL),*profile[1:-4],rl(profile[-4],RorL),*profile[-3:]]

def arms(baselink,name,profile,RorL):
    """
    profile: [[xyz,whd,mass,color,option1,option2,...],...]
    """
    linknames = [baselink] + [name+str(i+1) for i in range(len(profile))]
    profile = [changeRorL(p,RorL) for p in profile]

    for i,p in enumerate(profile):
        gs.link_box(*arm(linknames[i+1],*p[:-4]))
        #print(*linknames[i:i+2],*p[-4:])
        gs.joint_revolute(name+'r'+str(i),*linknames[i:i+2],*p[-4:])


#status
x = (1,0,0)
y = (0,1,0)
z = (0,0,1)
rpy = (0,0,0)
effort = 10
pi = 3.14
vel = 0.5
limits = (effort,-0.5*pi,0.5*pi,vel)

#body
gs.link_box('weist',(0,0,0),(0,0,0),(0.06,0.05,0.04),0.15,'Black')
gs.link_box('chest',(0,0,0.03),(0,0,0),(0.09,0.05,0.06),0.25,'White',selfcollide=True)
gs.joint_revolute('wc','weist','chest',(0,0.01,0.02),rpy,z,limits)
gs.link_box('head',(0,0,0.02),(0,0,0),(0.03,0.04,0.04),0.02,'White')
gs.joint_revolute('ch','chest','head',(0,0.01,0.06),rpy,z,limits)

gs.link_box('back',(0,-0.04,0.03),(0,0,0),(0.1,0.05,0.07),0.14,'Black',sensor='imu')
gs.joint_fixed('cb','chest','back')


arms_profile = [((0.02,0,0),(0.03,0.045,0.02),0.02,'Black',(0.045,0.01,0.045),(0,0,0),x,limits),
                ((0,0,-0.03),(0.02,0.03,0.08),0.1,'Black','selfcollide',(0.025,0,0),(0,0,0),y,limits),
                ((0,0,-0.015),(0.03,0.04,0.02),0.05,'Black',(0,0,-0.07),(0,0,0),z,limits),
                ((0,0,-0.05),(0.03,0.02,0.06),0.03,'White','selfcollide',(0,0.006,-0.015),(0,0,0),x,limits)]
arms('chest','Rarm',arms_profile,-1)
arms('chest','Larm',arms_profile,1)

#legs
legs_profile = [((0,0,-0.015),(0.02,0.05,0.03),0.015,'White',(0.023,0.01,-0.02),(0,0,0),z,(effort,-0.2*pi,0.2*pi,vel)),
                ((0.012,0,0),(0.04,0.03,0.02),0.065,'Black',(-0.006,0,-0.022),(0,0,0),y,(effort,-0.2*pi,0.2*pi,vel)),
                ((0,0,-0.03),(0.04,0.02,0.08),0.1,'Black','selfcollide',(0.012,0,-0.04),(0,0,0),x,limits),
                ((0,0,-0.04),(0.03,0.02,0.04),0.065,'Black',(0,0,-0.065),(0,0,0),x,limits),
                ((0,0,-0.02),(0.04,0.03,0.06),0.065,'Black',(0,0,-0.065),(0,0,0),x,limits),
                ((0.017,0,-0.01),(0.05,0.05,0.03),0.045,'White','selfcollide',(-0.012,0,-0.04),(0,0,0),y,(effort,-0.4*pi,0.4*pi,vel))]
arms('weist','Rleg',legs_profile,-1)
arms('weist','Lleg',legs_profile,1)

gs.output(os.getcwd().split('/')[-1])
