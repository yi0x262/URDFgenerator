#!/usr/env/bin python3
import lxml.etree import Element as element
import lxml.etree import SubElement as sub

class genLAUNCH(object):
    def __init__(self):
        self.launch = element('launch')

    def __str__(self):
        xmldec = '<?xml version="1.0"?>\n'
        launch = etree.tostring(self.launch,pretty_print=True,xml_declaration=False).decode('utf-8')
        return xmldec+urdf

    def controller_spawner(self,robotname,args):
        sub(self.launch,'node',
            name    = "controller",
            pkg     = "controller_manager",
            type    = "spawner",
            respawn = "false",
            output  = "screen",
            ns      = robotname,
            args    = str(args)[1:-1])

    def robot_state_publisher(self):
        sub(self.launch,'node',
            name    = "controller",
            pkg     = "controller_manager",
            type    = "spawner",
            respawn = "false",
            output  = "screen",
            ns      = robotname,
            args    = str(args)[1:-1])
