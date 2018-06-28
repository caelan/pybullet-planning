#!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.pr2_utils import PR2_GROUPS
from pybullet_tools.utils import HideOutput, disconnect, set_base_values, joint_from_name, connect, user_input, \
    dump_world, get_link_name, wait_for_interrupt, clone_body, get_link_parent, get_link_descendants, load_model

from lxml import etree
from sys import argv
import os

def writeShape(link):
    elem = link.find("origin")
    if elem is not None:
        if elem.find("rby") is not None:
            print('rel=<T t(%s) E(%s)>' % (elem.attrib['xyz'], elem.attrib['rpy']))
        else:
            print('rel=<T t(%s)>' % elem.attrib['xyz'])

    elem = link.find("geometry/box")
    if elem is not None:
        print('type=ST_box size=[%s 0]' % elem.attrib['size'])

    elem = link.find("geometry/sphere")
    if elem is not None:
        print('type=ST_sphere size=[0 0 0 %s]' % elem.attrib['radius'])

    elem = link.find("geometry/cylinder")
    if elem is not None:
        print('type=ST_cylinder size=[0 0 %s %s]' % (elem.attrib['length'], elem.attrib['radius']))

    elem = link.find("geometry/mesh")
    if elem is not None:
        print('type=ST_mesh mesh=\'%s\'' % elem.attrib['filename'])
        if elem.find("scale") is not None:
            print('meshscale=[%s]' % elem.attrib['scale'])

    elem = link.find("material/color")
    if elem is not None:
        print('color=[%s]' % elem.attrib['rgba'])

    elem = link.find("material")
    if elem is not None:
        if elem.attrib['name'] is not None:
            print('colorName=%s' % elem.attrib['name'])

def stuff(inFile):
    xmlData = etree.parse(inFile)
    print(xmlData)

    bodies = xmlData.findall("/KinBody")
    print(bodies)
    for body in bodies:
        name = body.attrib['name']
        print('body %s { X=<' % name)

        elem = body.find("Body/Translation")
        if elem is not None:
            print('t(%s)' % elem.text)

        elem = body.find("Body/rotationaxis")
        if elem is not None:
            rot = [float(i) for i in elem.text.split(' ')]
            print('d(%f %f %f %f)' % (rot[3], rot[0], rot[1], rot[2]))

        print('> }\n')  # end of body

        for geom in body.findall("Body/Geom"):
            print('shape (%s) {' % name)

            type = geom.attrib['type']
            if type == "box":
                size = [2 * float(i) for i in geom.find("extents").text.split(' ')]
                print('type=ST_box size=', size)

            elem = geom.find("translation")
            if elem is not None:
                print('rel=<t(%s)>' % elem.text)

            elem = geom.find("diffuseColor")
            if elem is not None:
                print('color=[%s]' % elem.text)

            print(' }\n')

# TODO: store things as JSONs?

def main():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    openrave_directory = os.path.join(root_directory, '..', 'openrave')
    rel_path = 'data/ikeatable.kinbody.xml'
    path = os.path.join(openrave_directory, rel_path)

    stuff(path)

    #connect(use_gui=True)

    #with HideOutput():
    #    pr2 = load_model("models/pr2_description/pr2.urdf")


    #user_input('Finish?')
    #disconnect()


if __name__ == '__main__':
    main()