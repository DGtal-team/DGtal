# Generated using: python setup_py_configure.py 'dgtal'

from __future__ import print_function
from os import sys, path

try:
    from skbuild import setup
except ImportError:
    print('scikit-build is required to build from source.', file=sys.stderr)
    print('Please run:', file=sys.stderr)
    print('', file=sys.stderr)
    print('  python -m pip install scikit-build')
    sys.exit(1)

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from dgtalVersion import get_versions

CMAKE_OPTIONS = [
    '-DCMAKE_BUILD_TYPE=Release',
    '-DDGTAL_BUILD_SHARED_LIBS:BOOL=OFF',
    '-DDGTAL_BUILD_EXAMPLES:BOOL=OFF',
    '-DDGTAL_BUILD_TESTS:BOOL=OFF',
    '-DDGTAL_WRAP_PYTHON:BOOL=ON'
]
if sys.platform == "win32":
    CMAKE_OPTIONS.append("-DENABLE_CONAN:BOOL=ON ")
    CMAKE_OPTIONS.append(" -DENABLE_CONAN=true")
    CMAKE_OPTIONS.append("-DCMAKE_C_COMPILER=\"cl.exe\"")
    CMAKE_OPTIONS.append("-DCMAKE_CXX_COMPILER=\"cl.exe\"")
    CMAKE_OPTIONS.append("-DCMAKE_TOOLCHAIN_FILE=\"conan_toolchain.cmake\"")
    CMAKE_OPTIONS.append("-DCMAKE_POLICY_DEFAULT_CMP0091=NEW")



# this_directory = path.abspath(path.dirname(__file__))
# dgtal_readme_path = path.join(this_directory, 'DGtal-source', 'DGtal', 'README.md')
# if path.exists(dgtal_readme_path):
#     with open(dgtal_readme_path, encoding='utf-8') as f:
#         long_description = f.read()
# else:
#     with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
#         long_description = f.read()

long_description= r'DGtal is an open-source, cross-platform library providing ' \
                   'Digital Geometry Tools and Algorithms.'
setup(
    name='DGtal',
    version=get_versions()['package-version'],
    author='David Coeurjolly',
    author_email='david.coeurjolly@cnrs.fr',
    packages=['dgtal'],
    package_dir={'dgtal': 'dgtal'},
    package_data={
        'dgtal': ['tables/*.zlib']
    },
    cmake_source_dir='../..', # Top CMakeLists.txt directory
    cmake_args=CMAKE_OPTIONS,
    cmake_install_target="dgtal-install-runtime",
    py_modules=[
        'dgtalVersion',
    ],
    download_url=r'https://github.com/DGtal-team/DGtal',
    description=r'Digital Geometry Tools and Algorithm Library',
    long_description=long_description,
    long_description_content_type='text/markdown',
    classifiers=[
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Programming Language :: Python",
        "Programming Language :: C++",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Information Analysis",
        "Topic :: Software Development :: Libraries",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX",
        "Operating System :: Unix",
        "Operating System :: MacOS"
        ],
    license='LGPLv3',
    keywords=r'DGtal image digital topology processing',
    url=r'https://github.com/DGtal-team/DGtal',
    install_requires=[
    ]
)
