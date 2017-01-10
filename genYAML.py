#!/usr/env/bin python3
import yaml

class genYAML(object):
    def __init__(self,robotname):
        self.joints = {}
        self.yaml = {robotname:self.joints}

    def __str__(self):
        return yaml.dump(self.yaml,default_flow_style=False)

    def joint_state_controller(self,publist_rate):
        info = {}
        info.update({'type':'joint_state_controller/JointStateController'})
        info.update({'publish_rate':publist_rate})
        jsc = {'joint_state_controller':info}
        self.joints.update(jsc)
        return 'joint_state_controller'

    def revolute_joint_controller(self,jointname,pid=(1,0,0)):
        #must : use jointtype argument
        controller_name = jointname+'_position_controller'
        info = {}
        info.update({'type':'effort_controllers/JointPositionController'})
        info.update({'joint':jointname})
        info.update({'pid':{'p':p,'i':i,'d':d}})#wanna this line flow_style...
        rjc = {controller_name:info}
        self.joints.update(rjc)
        return controller_name


if __name__ == '__main__':
    c = genYAML('robot')
    c.joint_state_controller(50)
    c.revolute_joint_controller('test')
    print(c.output())
    print(c.controllers)
