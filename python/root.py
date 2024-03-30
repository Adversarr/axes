import os

root_file_directory = os.path.dirname(os.path.abspath(__file__))
root_directory = os.path.abspath(f"{root_file_directory}/..")
AX_ROOT = root_directory

def axes_build_directory():
    return f"{AX_ROOT}/build"

def axes_source_directory():
    return f"{AX_ROOT}"

__all__ = [ 
  "AX_ROOT",
  "axes_build_directory",
  "axes_source_directory"
]