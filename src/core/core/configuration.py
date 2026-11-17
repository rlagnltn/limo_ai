from pathlib import Path
import os
import sys
from ament_index_python.packages import get_package_share_directory


def config_files():
    share = Path(get_package_share_directory('core'))
    source = share.parents[3] / 'src/core/config'
    directory = Path(os.environ['CORE_CONFIG_DIR']) if 'CORE_CONFIG_DIR' in os.environ else (
        source if source.is_dir() else share / 'config')
    return sorted(directory.glob('*.yaml'))


def ros_args(args=None):
    defaults = ['--ros-args']
    for path in config_files():
        defaults.extend(['--params-file', str(path)])
    return defaults + ['--'] + list(sys.argv[1:] if args is None else args)
