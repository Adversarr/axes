import os

root_file_directory = os.path.dirname(os.path.abspath(__file__))
root_directory = os.path.abspath(f"{root_file_directory}/..")
AXES_ROOT = root_directory

def axes_build_directory():
    return f"{AXES_ROOT}/build"

def axes_source_directory():
    return f"{AXES_ROOT}"

__all__ = [ 
  "AXES_ROOT",
  "axes_build_directory",
  "axes_source_directory"
]