#!/usr/bin/env python3
"""Rewrite package:// mesh URIs in a URDF to relative paths and copy the meshes next to it.

FOAM, Cricket and cuRobo read a URDF and its meshes from plain directories; they cannot follow ROS
package URIs. Needs a sourced ROS environment (ament index) for the packages the URDF mentions.

    resolve_package_uris.py in.urdf out_dir     # writes out_dir/robot.urdf and out_dir/<pkg>/...
Only <collision> meshes are kept in the output copy of the URDF's mesh set; <visual> elements are
dropped since the generators ignore them and the visual meshes can be large.
"""
import pathlib
import shutil
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def resolve(urdf_in, out_dir):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(urdf_in)
    copied = 0
    for link in tree.getroot().iter('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        for mesh in link.iter('mesh'):
            uri = mesh.get('filename', '')
            if not uri.startswith('package://'):
                continue
            pkg, _, rel = uri[len('package://'):].partition('/')
            src = pathlib.Path(get_package_share_directory(pkg)) / rel
            if not src.is_file():
                raise FileNotFoundError(f'{uri} -> {src} does not exist')
            dst = out / pkg / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            mesh.set('filename', f'{pkg}/{rel}')
            copied += 1
    tree.write(out / 'robot.urdf', xml_declaration=True, encoding='utf-8')
    return copied


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    print(f'resolved {resolve(sys.argv[1], sys.argv[2])} meshes')
