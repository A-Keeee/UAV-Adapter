#!/usr/bin/python3
"""Make a per-run copy of the existing world with the RGL scene manager enabled."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    tree = ET.parse(args.source, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    world = tree.getroot().find('world')
    if world is None:
        raise SystemExit('Expected an SDF world')
    if world.find("plugin[@name='rgl::RGLServerPluginManager']") is not None:
        raise SystemExit('RGL scene manager already exists')
    plugin = ET.SubElement(world, 'plugin', filename='RGLServerPluginManager', name='rgl::RGLServerPluginManager')
    ET.SubElement(plugin, 'do_ignore_entities_in_lidar_link').text = 'true'
    ET.indent(tree, space='  ')
    tree.write(args.output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
