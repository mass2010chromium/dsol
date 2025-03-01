from setuptools import setup, Extension

all_macros = [('PYTHON', None)]

# TODO: Should this be auto detected?
cv_version = "cv4"

import os
file_dir = os.path.dirname(os.path.abspath(__file__))
dsol_root = os.path.normpath(os.path.join(file_dir, '..'))
pyboostconverter_root = os.path.join(file_dir, 'pyboostcvconverter', 'include')
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
    "dataset.cpp",  # is this needed?
    "eigen.cpp",
    "math.cpp",
    "metric.cpp",
    "ocv.cpp",
    "summary.cpp",
]]]

absl_libs = [
"absl_base",
"absl_city",
"absl_civil_time",
"absl_cord",
"absl_cord_internal",
"absl_cordz_functions",
"absl_cordz_handle",
"absl_cordz_info",
"absl_cordz_sample_token",
"absl_crc32c",
"absl_crc_cord_state",
"absl_crc_cpu_detect",
"absl_crc_internal",
"absl_debugging_internal",
"absl_decode_rust_punycode",
"absl_demangle_internal",
"absl_demangle_rust",
"absl_die_if_null",
"absl_examine_stack",
"absl_exponential_biased",
"absl_failure_signal_handler",
"absl_flags_commandlineflag",
"absl_flags_commandlineflag_internal",
"absl_flags_config",
"absl_flags_internal",
"absl_flags_marshalling",
"absl_flags_parse",
"absl_flags_private_handle_accessor",
"absl_flags_program_name",
"absl_flags_reflection",
"absl_flags_usage",
"absl_flags_usage_internal",
"absl_graphcycles_internal",
"absl_hash",
"absl_hashtablez_sampler",
"absl_int128",
"absl_kernel_timeout_internal",
"absl_leak_check",
"absl_log_flags",
"absl_log_globals",
"absl_log_initialize",
"absl_log_internal_check_op",
"absl_log_internal_conditions",
"absl_log_internal_fnmatch",
"absl_log_internal_format",
"absl_log_internal_globals",
"absl_log_internal_log_sink_set",
"absl_log_internal_message",
"absl_log_internal_nullguard",
"absl_log_internal_proto",
"absl_log_internal_structured_proto",
"absl_log_severity",
"absl_log_sink",
"absl_low_level_hash",
"absl_malloc_internal",
"absl_periodic_sampler",
"absl_poison",
"absl_random_distributions",
"absl_random_internal_distribution_test_util",
"absl_random_internal_platform",
"absl_random_internal_pool_urbg",
"absl_random_internal_randen",
"absl_random_internal_randen_hwaes",
"absl_random_internal_randen_hwaes_impl",
"absl_random_internal_randen_slow",
"absl_random_internal_seed_material",
"absl_random_seed_gen_exception",
"absl_random_seed_sequences",
"absl_raw_hash_set",
"absl_raw_logging_internal",
"absl_scoped_set_env",
"absl_spinlock_wait",
"absl_stacktrace",
"absl_status",
"absl_statusor",
"absl_strerror",
"absl_str_format_internal",
"absl_strings",
"absl_strings_internal",
"absl_string_view",
"absl_symbolize",
"absl_synchronization",
"absl_throw_delegate",
"absl_time",
"absl_time_zone",
"absl_tracing_internal",
"absl_utf8_for_code_point",
"absl_vlog_config_internal",
]

import numpy

dsol = Extension('dsol',
                 sources = [
                    'c++/dsol_py.cpp',
                    f'pyboostcvconverter/src/pyboost_{cv_version}_converter.cpp'
                 ] + dsol_sources,
                 include_dirs=[
                    dsol_root,
                    pyboostconverter_root,
                    numpy.get_include()
                 ] + opencv_include_dirs + eigen_include_dirs,
                 libraries=['opencv_core'] + absl_libs,
                 extra_compile_args = ["--std=c++17", "-O3"],
                 define_macros = all_macros,
                 extra_link_args=[])

setup (name = 'DSOL',
       version = '1.0',
       description = 'DSOL python bindings',
       ext_modules = [dsol])
