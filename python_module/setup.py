from setuptools import setup, Extension

all_macros = [('PYTHON', None), ('GOOGLE_STRIP_LOG', 1)]

# NOTE: this code only works with cv4
cv_version = "cv4"

import os
file_dir = os.path.dirname(os.path.abspath(__file__))
dsol_root = os.path.normpath(os.path.join(file_dir, '..'))
# OpenCV likes to be in /usr/include/opencv4/opencv2 :doom:
opencv_include_dirs = ['/usr/include/opencv4']
# Eigen also likes to be in /usr/include/eigen3/Eigen
eigen_include_dirs = ['/usr/include/eigen3']

dsol_sources = [os.path.join(dsol_root, 'sv', y) for y in [os.path.join('dsol', x) for x in [
    "adjust.cpp",
    "align.cpp",
    "camera.cpp",
    "direct.cpp",
    "extra.cpp",
    "frame.cpp",
    "hessian.cpp",
    "image.cpp",
    "odom.cpp",
    "point.cpp",
    "select.cpp",
    "solve.cpp",
    "stereo.cpp",
    "viz.cpp",
    "window.cpp",
]] + [os.path.join('util', y) for y in [
    "cmap.cpp",
    #"dataset.cpp",  # is this needed?
    "eigen.cpp",
    "math.cpp",
    #"metric.cpp",
    "ocv.cpp",
    "summary.cpp",
]]]

absl_libs = [
"absl_cord",
"absl_cordz_info",
"absl_cord_internal",
"absl_cordz_functions",
"absl_cordz_handle",
"absl_hash",
"absl_city",
"absl_bad_variant_access",
"absl_low_level_hash",
"absl_raw_hash_set",
"absl_bad_optional_access",
"absl_hashtablez_sampler",
"absl_exponential_biased",
"absl_synchronization",
"absl_stacktrace",
"absl_graphcycles_internal",
"absl_symbolize",
"absl_debugging_internal",
"absl_malloc_internal",
"absl_demangle_internal",
"absl_time",
"absl_strings",
"absl_strings_internal",
"absl_base",
"absl_spinlock_wait",
"absl_civil_time",
"absl_int128",
"absl_time_zone",
"absl_throw_delegate",
"absl_raw_logging_internal",
"absl_log_severity",
]

#absl_objects = [ f"/usr/local/lib/lib{n}.a" for n in absl_libs ]
absl_objects = [ f"/usr/lib/x86_64-linux-gnu/lib{n}.a" for n in absl_libs ]

#fmt_objects = ["/usr/local/lib/libfmt.a"]
fmt_objects = ["/usr/lib/x86_64-linux-gnu/libfmt.so"]

import numpy

# Not sure if google benchmark library is a link dependency
dsol = Extension('dsol',
                 sources = [
                    'c++/dsol_py.cpp',
                    'c++/pyboost_cv4_converter.cpp'
                 ] + dsol_sources,
                 include_dirs=[
                    "/opt/ros/humble/include",
                    dsol_root,
                    numpy.get_include()
                 ] + opencv_include_dirs + eigen_include_dirs,
                 #libraries=['opencv_core', 'opencv_imgproc', 'opencv_highgui', 'glog', 'benchmark'],
                 #libraries=['opencv_core', 'opencv_imgproc', 'opencv_highgui', 'glog', 'benchmark', 'tbb'],
                 libraries=['opencv_core', 'opencv_imgproc', 'opencv_highgui', 'glog', 'tbb'],
                 library_dirs=[
                    "/usr/local/lib",
                    #"/usr/local/lib/x86_64-linux-gnu",
                 ],
                 extra_compile_args = ["--std=c++20", "-O3"],
                 define_macros = all_macros,
                 extra_link_args=[],
                 #extra_objects=fmt_objects + absl_objects)
                 extra_objects=fmt_objects)

setup (name = 'DSOL',
       version = '1.0',
       description = 'DSOL python bindings',
       ext_modules = [dsol])
