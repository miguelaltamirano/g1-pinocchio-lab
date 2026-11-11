"""Pack official URDF/STL visuals for the browser (instructor-only; needs NumPy).

Usage: python scripts/build_robot_assets.py /path/to/models
The directory must contain g1_original.urdf and its meshes/ subdirectory.
Preserves every triangle, joint origin, visual origin and mesh scale.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('models', type=Path)
args = p.parse_args()
out = Path(__file__).resolve().parents[1] / 'dist/assets/g1'
out.mkdir(parents=True, exist_ok=True)
source = args.models / 'g1_original.urdf'
robot = ET.parse(source).getroot()


def values(element, name, default):
    return [float(x) for x in (element.get(name, default) if element is not None else default).split()]


def origin(element):
    return {'xyz': values(element, 'xyz', '0 0 0'), 'rpy': values(element, 'rpy', '0 0 0')}


blob = bytearray()
meshes = {}
for mesh in robot.findall('.//visual/geometry/mesh'):
    name = mesh.get('filename')
    if name in meshes:
        continue
    raw = (args.models / name).read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    assert len(raw) == 84 + count * 50
    triangles = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')]), offset=84)
    vertices, indices = np.unique(triangles['vertices'].reshape(-1, 3), axis=0, return_inverse=True)
    vertex_offset = len(blob)
    blob.extend(vertices.astype('<f4').tobytes())
    index_offset = len(blob)
    blob.extend(indices.astype('<u4').tobytes())
    meshes[name] = {'vertexOffset': vertex_offset, 'vertexCount': len(vertices),
                    'indexOffset': index_offset, 'indexCount': len(indices),
                    'sourceSha256': hashlib.sha256(raw).hexdigest()}

materials = {m.get('name'): values(m.find('color'), 'rgba', '0.7 0.7 0.7 1') for m in robot.findall('material')}
links = []
for link in robot.findall('link'):
    visuals = []
    for visual in link.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            raise ValueError('Unexpected non-mesh visual')
        material = visual.find('material')
        visuals.append({**origin(visual.find('origin')), 'mesh': mesh.get('filename'),
                        'scale': values(mesh, 'scale', '1 1 1'),
                        'rgba': materials[material.get('name')]})
    links.append({'name': link.get('name'), 'visuals': visuals})
joints = [{**origin(j.find('origin')), 'name': j.get('name'), 'type': j.get('type'),
           'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
           'axis': values(j.find('axis'), 'xyz', '1 0 0')} for j in robot.findall('joint')]
model = {'source': 'unitreerobotics/unitree_ros@9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90',
         'urdfSha256': hashlib.sha256(source.read_bytes()).hexdigest(),
         'meshes': meshes, 'links': links, 'joints': joints}
(out / 'robot.json').write_text(json.dumps(model, separators=(',', ':')) + '\n')
(out / 'meshes.bin').write_bytes(blob)
print(f'{len(meshes)} meshes, {sum(m["indexCount"] // 3 for m in meshes.values())} triangles, {len(blob) / 1e6:.2f} MB')
