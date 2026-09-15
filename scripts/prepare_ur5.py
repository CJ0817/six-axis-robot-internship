"""Regenerate vendored UR5 from an exact upstream checkout; no ROS required."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import xacro

ROOT = Path(__file__).resolve().parents[1]
LOCK = json.loads((ROOT / 'environment.lock.json').read_text())['robot']

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True, help='universal_robot checkout')
    args = parser.parse_args()
    source = args.source.resolve()
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != LOCK['commit']:
        raise ValueError(f'Wrong upstream commit: {actual}')
    if subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain', '--', 'ur_description'], text=True).strip():
        raise ValueError('Upstream ur_description must be unmodified')
    destination = ROOT / 'models/ur5'
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temp:
        package = Path(temp) / 'ur_description'
        # Only the xacro includes and UR5 configuration are needed for expansion.
        shutil.copytree(source / 'ur_description/urdf', package / 'urdf')
        shutil.copytree(source / 'ur_description/config/ur5', package / 'config/ur5')
        for path in package.rglob('*.xacro'):
            path.write_text(path.read_text().replace('$(find ur_description)', str(package)))
        document = xacro.process_file(str(package / 'urdf/ur5.xacro'))
        tree = ET.fromstring(document.toxml())
        # PyBullet does not use the ROS transmission tags. Keep them as source metadata.
        # Use upstream collision STL also for visuals (DAE-free, simplified appearance).
        for link in tree.findall('link'):
            visual, collision = link.find('visual'), link.find('collision')
            if visual is not None and collision is not None:
                for tag in ('origin', 'geometry'):
                    old = visual.find(tag)
                    if old is not None:
                        visual.remove(old)
                    new = collision.find(tag)
                    if new is not None:
                        visual.append(copy.deepcopy(new))
        # ROS coordinate-only links have no inertia. PyBullet otherwise assigns 1 kg.
        for link in tree.findall('link'):
            if link.find('inertial') is None:
                if link.attrib['name'] not in ('base_link', 'base', 'flange', 'tool0'):
                    raise ValueError('Unexpected missing inertia: ' + link.attrib['name'])
                inertial = ET.SubElement(link, 'inertial')
                ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
                ET.SubElement(inertial, 'mass', value='0')
                ET.SubElement(inertial, 'inertia', ixx='0', ixy='0', ixz='0', iyy='0', iyz='0', izz='0')
        for mesh in tree.findall('.//mesh'):
            original = mesh.attrib['filename']
            prefix = 'package://ur_description/meshes/ur5/collision/'
            if not original.startswith(prefix):
                raise ValueError(f'Unexpected mesh: {original}')
            name = original[len(prefix):]
            (destination / 'meshes').mkdir(exist_ok=True)
            shutil.copyfile(source / 'ur_description/meshes/ur5/collision' / name,
                            destination / 'meshes' / name)
            mesh.set('filename', 'meshes/' + name)
        ET.indent(tree, space='  ')
        (destination / 'ur5.urdf').write_text('<?xml version="1.0"?>\n<!-- Generated from pinned ROS-Industrial UR5; see NOTICE.md and manifest.json. -->\n' + ET.tostring(tree, encoding='unicode') + '\n')
    shutil.copyfile(source / 'ur_description/LICENSE', destination / 'LICENSE')
    hashes = {str(path.relative_to(destination)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(destination.rglob('*')) if path.is_file() and path.suffix in ('.urdf', '.stl')}
    manifest = {'upstream_commit': actual, 'source': LOCK['source'], 'sha256': hashes,
                'adaptations': ['resolve package mesh paths to local relative paths',
                                'use unchanged collision STL and collision origins for simplified visual geometry',
                                'declare four coordinate-only fixed links massless to prevent PyBullet default 1 kg inertia'],
                'kinematics_inertia_limits': 'unchanged from upstream generic UR5 configuration'}
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'urdf': str(destination / 'ur5.urdf'), 'assets': len(hashes), 'upstream_commit': actual}))

if __name__ == '__main__':
    main()
