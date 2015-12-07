from distutils.core import setup
import py2exe, cv2, sys, os

sys.argv.append('py2exe')

setup(
    console=['testOpenCV.py'],
)

# setup(
#     #console=['testOpenCV.py'],
#     options = {'py2exe': {'bundle_files': 1, 'compressed': True}},
#     windows = [{'script': 'testOpenCV.py'}],
#     zipfile = None,
# )

