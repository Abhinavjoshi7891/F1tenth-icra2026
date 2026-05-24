import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def get_package_path():
    """
    Resolves the package path to the source folder if available (so we can write model/LUT updates directly),
    otherwise falls back to the share folder.
    """
    try:
        share_dir = get_package_share_directory('on_track_sys_id')
        # Search upward from share_dir for the source tree — robust across any colcon workspace layout.
        for parent in Path(share_dir).parents:
            candidate = parent / 'race_stack' / 'system_identification' / 'on_track_sys_id'
            if candidate.exists():
                return str(candidate)
            candidate = parent / 'src' / 'forzaeth' / 'race_stack' / 'system_identification' / 'on_track_sys_id'
            if candidate.exists():
                return str(candidate)
        return share_dir
    except Exception:
        # Fallback to absolute path relative to this file
        current_file_path = Path(__file__).resolve()
        package_root = current_file_path.parents[2]
        return str(package_root)
