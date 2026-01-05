import os
from os.path import dirname
from ament_index_python.packages import get_package_share_directory

def get_pkg_src_path(jackal_pkg = False):
    # hack to get ws/src/The-Barn-Challenge-Ros2(/jackal_helper)
    workspace_path = dirname(dirname(dirname(dirname(get_package_share_directory("jackal_helper")))))
    BARN_challenge_src_path = os.path.join(workspace_path, "src", "The-Barn-Challenge-Ros2")
    if jackal_pkg:
        return os.path.join(BARN_challenge_src_path, "jackal_helper")
    return BARN_challenge_src_path
