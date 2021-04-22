#!/usr/bin/env python

from __future__ import print_function

from lxml import etree # pip install lxml
import os

def find_insensitive(elem, pattern):
    result1 = elem.find(pattern)
    if result1 is not None:
        return result1
    return elem.find(pattern.lower())

# https://github.mit.edu/mtoussai/KOMO-stream/blob/master/01-basicKOMOinTheKitchen/env2rai.py
# https://github.mit.edu/mtoussai/KOMO-stream/blob/master/01-basicKOMOinTheKitchen/collada2rai.py
# http://openrave.programmingvision.com/wiki/index.php/Format:XML
# http://wiki.ros.org/collada_urdf
# http://openrave.org/docs/latest_stable/collada_robot_extensions/
# http://openrave.org/docs/0.8.2/robots_overview/#openrave-xml

def parse_shape(link):
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

# TODO: this is case sensitive...
def parse_body(body):
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
            size = [2 * float(i) for i in find_insensitive(geom, "Extents").text.split(' ')]
            print('type=ST_box size=', size)
        else:
            pass
            #raise NotImplementedError(type)
        elem = geom.find("translation")
        if elem is not None:
            print('rel=<t(%s)>' % elem.text)
        elem = geom.find("diffuseColor")
        if elem is not None:
            print('color=[%s]' % elem.text)
        print(' }\n')

def parse_path(path):

    xmlData = etree.parse(path)
    print(xmlData)

    root = xmlData.getroot()
    parse_body(root)

    #bodies = xmlData.find("/Environment")
    #print(bodies)

    #bodies = xmlData.findall("/KinBody")
    #print(bodies)
    #for body in bodies:
    #    parse_body(body)

# TODO: store things as JSONs?

def main():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    openrave_directory = os.path.join(root_directory, '..', 'openrave')
    rel_path = 'data/ikeatable.kinbody.xml'
    path = os.path.join(openrave_directory, rel_path)

    parse_path(path)

    #connect(use_gui=True)
    #wait_if_gui('Finish?')
    #disconnect()


if __name__ == '__main__':
    main()