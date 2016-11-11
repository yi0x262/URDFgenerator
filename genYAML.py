#!/usr/env/bin python3
import yaml

class genYAML(object):
    def __init__(self,robotname):
        self.joints = {}
        self.yaml = {robotname:self.joints}
        self.controllers = []

    def output(self):
        return yaml.dump(self.yaml,default_flow_style=False)

    def joint_state_controller(self,publist_rate):
        info = {}
        info.update({'type':'joint_state_controller/JointStateController'})
        info.update({'publish_rate':publist_rate})
        jsc = {'joint_state_controller':info}
        self.joints.update(jsc)
        self.controllers.append('joint_state_controller')

    def revolute_joint_controller(self,jointname,pid={'p':1,'i':0,'d':0}):
        #must : use jointtype argument
        info = {}
        info.update({'type':'effort_controllers/JointPositionController'})
        info.update({'joint':jointname})
        info.update({'pid':pid})#wanna this line flow_style...
        rjc = {jointname+'_position_controller':info}
        self.joints.update(rjc)
        self.controllers.append(jointname+'_position_controller')


if __name__ == '__main__':
    c = genYAML('robot')
    c.joint_state_controller(50)
    c.revolute_joint_controller('test')
    print(c.output())
    print(c.controllers)
