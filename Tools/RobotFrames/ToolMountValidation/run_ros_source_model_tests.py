#!/usr/bin/env python3
"""Run actual source launch-model tests in a network-isolated existing ROS image.

The checkout is mounted read-only at /source. Only a temporary package index,
generated control wrapper and pytest artifacts are written. No LaunchService or
Node action executes; test_model_consistency.py observes construction only.
"""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


SOURCE = Path('/source')
OUTPUT = Path('/validation')


def main():
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    from ament_index_python.packages import get_package_share_directory

    driver = Path(get_package_share_directory('ur_robot_driver')) / 'urdf/ur.ros2_control.xacro'
    with tempfile.TemporaryDirectory(prefix='source-model-prefix-') as temporary:
        prefix = Path(temporary)
        packages = []
        for manifest in sorted((SOURCE / 'ros_ws/src').glob('*/package.xml')):
            package = ET.parse(manifest).getroot().findtext('name')
            package_source = manifest.parent
            share = prefix / 'share' / package
            share.parent.mkdir(parents=True, exist_ok=True)
            resource = prefix / 'share/ament_index/resource_index/packages' / package
            resource.parent.mkdir(parents=True, exist_ok=True)
            resource.touch()
            packages.append(package)
            if package == 'ur16e_mir250_control':
                shutil.copytree(package_source, share)
                spec = importlib.util.spec_from_file_location(
                    'source_control_generator', share / 'scripts/generate_control_xacro.py')
                generator = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(generator)
                wrapper = generator.render_control_xacro(
                    driver, (share / 'urdf/ur16e_mir250.urdf.xacro.in').read_text())
                (share / 'urdf/ur16e_mir250.urdf.xacro').write_text(wrapper)
            else:
                share.symlink_to(package_source, target_is_directory=True)
        os.environ['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')

        import pytest
        import xacro
        import yaml
        result = pytest.main([
            '-q', '-p', 'no:cacheprovider',
            '--basetemp=' + str(prefix / 'pytest-temp'),
            '--junitxml=' + str(OUTPUT / 'ros-source-model-results.xml'),
            str(SOURCE / 'ros_ws/src/ur16e_bringup/test/test_model_consistency.py'),
        ])
        report = {
            'pytestExitCode': int(result),
            'sourceReadOnly': True,
            'networkMode': 'none',
            'rosNodeActionsExecuted': False,
            'robotCommandsSent': False,
            'packageOverlay': packages,
            'driverMacro': str(driver),
            'xacroModule': xacro.__file__,
            'yamlVersion': yaml.__version__,
            'junit': 'ros-source-model-results.xml',
        }
        (OUTPUT / 'ros-source-model-test-summary.json').write_text(json.dumps(report, indent=2) + '\n')
        return int(result)


if __name__ == '__main__':
    raise SystemExit(main())
