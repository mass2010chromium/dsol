#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API

#include <Python.h>
#include "structmember.h"
#include <Eigen/Core>
#include <pyboostcvconverter/pyboostcvconverter.hpp>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#define motion_dtype double
#include <motionlib/se3.h>

#include <stdio.h>

#include "sv/dsol/extra.h"
#include "sv/dsol/odom.h"

typedef struct {
    sv::dsol::MotionModel motion;
    sv::dsol::DirectOdometry odom;
} Odometry;

typedef struct {
    PyObject_HEAD
    double track_time;
    double frame_time;
    double w[3];
    Sophus::SO3d integrate_rot;
    bool has_gyro;
    bool has_time;
    Odometry* refs;
} OdometryObject;

static void
Odometry_dealloc(OdometryObject* self) {
    // Can this be a no-op? I'm not confident in the C++ destructors being called.
    delete self->refs;
    Py_TYPE(self)->tp_free((PyObject*) self);
}

static PyObject*
Odometry_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    OdometryObject* self = (OdometryObject*) type->tp_alloc(type, 0);
    self->refs = new Odometry();
    self->track_time = 0;
    self->frame_time = 0;
    self->w[0] = 0;
    self->w[1] = 0;
    self->w[2] = 0;
    self->integrate_rot = Sophus::SO3d();
    self->has_gyro = false;
    self->has_time = false;
    return (PyObject*) self;
}

static PyObject*
get_dict_or_err(PyObject* dict, const char* key) {
    PyObject* res = PyDict_GetItemString(dict, key);
    if (res == NULL) {
        PyErr_Format(PyExc_ValueError, "Missing Key %s (see dsol configuration)", key);
        return NULL;
    }
    if (!PyDict_Check(res)) {
        PyErr_Format(PyExc_TypeError, "Expected item %s to be a dictionary", key);
        return NULL;
    }
    return res;
}

static int
get_num_or_err(PyObject* dict, const char* key, double* ret) {
    PyObject* res = PyDict_GetItemString(dict, key);
    if (res == NULL) {
        PyErr_Format(PyExc_ValueError, "Missing Key %s (see dsol configuration)", key);
        return -1;
    }
    if (!(PyFloat_Check(res) || PyLong_Check(res))) {
        PyErr_Format(PyExc_TypeError, "Expected numeric value for key %s", key);
        return -1;
    }
    *ret = PyFloat_AsDouble(res);
    return 0;
}


static int
get_int_or_default(PyObject* dict, const char* key, int* ret, int default_val) {
    PyObject* res = PyDict_GetItemString(dict, key);
    if (res == NULL) {
        *ret = default_val;
        return 0;
    }
    if (!PyLong_Check(res)) {
        PyErr_Format(PyExc_TypeError, "Expected integer value for key %s", key);
        return -1;
    }
    *ret = PyLong_AsLong(res);
    return 0;
}

static int
get_int_or_err(PyObject* dict, const char* key, int* ret) {
    PyObject* res = PyDict_GetItemString(dict, key);
    if (res == NULL) {
        PyErr_Format(PyExc_ValueError, "Missing Key %s (see dsol configuration)", key);
        return -1;
    }
    if (!PyLong_Check(res)) {
        PyErr_Format(PyExc_TypeError, "Expected integer value for key %s", key);
        return -1;
    }
    *ret = PyLong_AsLong(res);
    return 0;
}

static int
get_bool_or_err(PyObject* dict, const char* key, bool* ret) {
    PyObject* res = PyDict_GetItemString(dict, key);
    if (res == NULL) {
        PyErr_Format(PyExc_ValueError, "Missing Key %s (see dsol configuration)", key);
        return -1;
    }
    *ret = PyObject_IsTrue(res);
    return 0;
}

static int
Odometry_init(OdometryObject* self, PyObject* args, PyObject* kwds) {
    sv::dsol::DirectOdometry& odom = self->refs->odom;
    sv::dsol::MotionModel& motion = self->refs->motion;

    double width;
    double height;
    double camera_p[12];
    PyObject* config;
    if (!PyArg_ParseTuple(args, "dd((dddd)(dddd)(dddd))O",
        &width, &height,
        camera_p+0, camera_p+1, camera_p+2,  camera_p+3,
        camera_p+4, camera_p+5, camera_p+6,  camera_p+7,
        camera_p+8, camera_p+9, camera_p+10, camera_p+11,
        &config
    )) {
        return -1;
    }
    if (!PyDict_Check(config)) { PyErr_SetString(PyExc_TypeError, "config should be a dictionary"); return -1; }
    PyObject* odom_cfg = get_dict_or_err(config, "odom");
    PyObject* sel_cfg = get_dict_or_err(config, "select");
    PyObject* stereo_cfg = get_dict_or_err(config, "stereo");
    PyObject* align_cfg = get_dict_or_err(config, "align");
    PyObject* adjust_cfg = get_dict_or_err(config, "adjust");
    if (odom_cfg == NULL || sel_cfg == NULL || stereo_cfg == NULL || align_cfg == NULL || adjust_cfg == NULL) {
        return -1;
    }


    {
        sv::dsol::OdomCfg cfg;
        if (get_bool_or_err(odom_cfg, "marg", &cfg.marg) ||
            get_int_or_err(odom_cfg, "num_kfs", &cfg.num_kfs) ||
            get_int_or_err(odom_cfg, "num_levels", &cfg.num_levels) ||
            get_num_or_err(odom_cfg, "min_track_ratio", &cfg.min_track_ratio) ||
            get_num_or_err(odom_cfg, "vis_min_depth", &cfg.vis_min_depth) ||

            get_bool_or_err(odom_cfg, "reinit", &cfg.reinit) ||
            get_bool_or_err(odom_cfg, "init_depth", &cfg.init_depth) ||
            get_bool_or_err(odom_cfg, "init_stereo", &cfg.init_stereo) ||
            get_bool_or_err(odom_cfg, "init_align", &cfg.init_align) ||

            get_int_or_default(config, "tbb", &cfg.tbb, 0) ||
            get_int_or_default(config, "log", &cfg.log, 5) ||
            get_int_or_default(config, "vis", &cfg.vis, 0)) { return -1; }
        odom.Init(cfg);
    }
    {
        sv::dsol::SelectCfg cfg;
        if (get_int_or_err(sel_cfg, "sel_level", &cfg.sel_level) ||
            get_int_or_err(sel_cfg, "cell_size", &cfg.cell_size) ||
            get_int_or_err(sel_cfg, "min_grad", &cfg.min_grad) ||
            get_int_or_err(sel_cfg, "max_grad", &cfg.max_grad) ||
            get_int_or_err(sel_cfg, "nms_size", &cfg.nms_size) ||
            get_num_or_err(sel_cfg, "min_ratio", &cfg.min_ratio) ||
            get_num_or_err(sel_cfg, "max_ratio", &cfg.max_ratio) ||
            get_bool_or_err(sel_cfg, "reselect", &cfg.reselect)) { return -1; }
        odom.selector = sv::dsol::PixelSelector(cfg);
    }
    {
        sv::dsol::DirectCfg cfg;
        if (get_int_or_err(align_cfg, "init_level", &cfg.optm.init_level) ||
            get_int_or_err(align_cfg, "max_iters", &cfg.optm.max_iters) ||
            get_num_or_err(align_cfg, "max_xs", &cfg.optm.max_xs) ||
            get_bool_or_err(align_cfg, "affine", &cfg.cost.affine) ||
            get_bool_or_err(align_cfg, "stereo", &cfg.cost.stereo) ||
            get_int_or_err(align_cfg, "c2", &cfg.cost.c2) ||
            get_int_or_err(align_cfg, "dof", &cfg.cost.dof) ||
            get_int_or_err(align_cfg, "max_outliers", &cfg.cost.max_outliers) ||
            get_num_or_err(align_cfg, "grad_factor", &cfg.cost.grad_factor) ||
            get_num_or_err(align_cfg, "min_depth", &cfg.cost.min_depth)) { return -1; }
        odom.aligner = sv::dsol::FrameAligner(cfg);
    }
    {
        sv::dsol::DirectCfg cfg;
        if (get_int_or_err(adjust_cfg, "init_level", &cfg.optm.init_level) ||
            get_int_or_err(adjust_cfg, "max_iters", &cfg.optm.max_iters) ||
            get_num_or_err(adjust_cfg, "max_xs", &cfg.optm.max_xs) ||
            get_bool_or_err(adjust_cfg, "affine", &cfg.cost.affine) ||
            get_bool_or_err(adjust_cfg, "stereo", &cfg.cost.stereo) ||
            get_int_or_err(adjust_cfg, "c2", &cfg.cost.c2) ||
            get_int_or_err(adjust_cfg, "dof", &cfg.cost.dof) ||
            get_int_or_err(adjust_cfg, "max_outliers", &cfg.cost.max_outliers) ||
            get_num_or_err(adjust_cfg, "grad_factor", &cfg.cost.grad_factor) ||
            get_num_or_err(adjust_cfg, "min_depth", &cfg.cost.min_depth)) { return -1; }
        odom.adjuster = sv::dsol::BundleAdjuster(cfg);
    }
    {
        sv::dsol::StereoCfg cfg;
        if (get_int_or_err(stereo_cfg, "half_rows", &cfg.half_rows) ||
            get_int_or_err(stereo_cfg, "half_cols", &cfg.half_cols) ||
            get_int_or_err(stereo_cfg, "match_level", &cfg.match_level) ||
            get_int_or_err(stereo_cfg, "refine_size", &cfg.refine_size) ||
            get_num_or_err(stereo_cfg, "min_zncc", &cfg.min_zncc) ||
            get_num_or_err(stereo_cfg, "min_depth", &cfg.min_depth)) { return -1; }
        odom.matcher = sv::dsol::StereoMatcher(cfg);
    }
    motion.Init();

    cv::Size size(width, height);
    Eigen::Array4d fc;
    fc << camera_p[0], camera_p[5], camera_p[2], camera_p[6];
    odom.camera = {size, fc, -camera_p[3]/camera_p[0]};
    printf("%f %f %f %f %f %f\n", width, height, camera_p[0], camera_p[5], camera_p[2], camera_p[6]);

    return 0;
}

static PyObject*
Odometry_input_gyro(OdometryObject* self, PyObject* args, Py_ssize_t nargs) {

    double t;
    double w[3];
    if (!PyArg_ParseTuple(args, "(ddd)d", w+0, w+1, w+2, &t)) {
        return NULL;
    }
    
    // NOTE: Following DSOL and doing 0th order integration.
    double dt = t - self->track_time;
    Eigen::Map<const Eigen::Vector3d> prev_w(self->w);
    self->integrate_rot *= Sophus::SO3d::exp(prev_w * dt);

    self->w[0] = w[0];
    self->w[1] = w[1];
    self->w[2] = w[2];
    self->track_time = t;
    self->has_gyro = true;
    if (!self->has_time) {
        self->frame_time = self->track_time;
    }
    self->has_time = true;
    Py_RETURN_NONE;
}

static int
track_frame(OdometryObject* self, double t,
        const cv::Mat& image_left, const cv::Mat& image_right, const cv::Mat& depth) {

    // Only update motion model if time has been recorded.
    double dt = 0;
    Sophus::SE3d dtf_pred;
    if (self->has_time) {
        dt = t - self->frame_time;
        dtf_pred = self->refs->motion.PredictDelta(dt);
        if (self->has_gyro) {
            // Integrate gyro measurement.
            // Following DSOL, we just overwrite the predicted angular velocity.
            dtf_pred.so3() = self->integrate_rot;
            self->integrate_rot = Sophus::SO3d();
            self->has_gyro = false;
        }
    }
    sv::dsol::OdomStatus status = self->refs->odom.Estimate(image_left, image_right, dtf_pred, depth);
    self->has_time = true;
    self->frame_time = t;
    if (status.track.ok) {
        self->refs->motion.Correct(status.Twc(), dt);
        return 0;
    } else {
        fprintf(stderr, "DSOL tracking failed (or was first frame)");
        return 1;
    }
}

static PyObject*
Odometry_input_monocular(OdometryObject* self, PyObject* const* args, Py_ssize_t nargs) {
    if (nargs != 2) {
        PyErr_SetString(PyExc_TypeError, "Invalid number of arguments. Expected: [image, timestamp]");
        return NULL;
    }
    PyObject* img = args[0];
    double t = PyFloat_AsDouble(args[1]);
    cv::Mat mat_left = pbcvt::fromNDArrayToMat(img);
    cv::Mat mat_right;
    cv::Mat mat_depth;
    int res = track_frame(self, t, mat_left, mat_right, mat_depth);
    return PyLong_FromLong(res);
}

static PyObject*
Odometry_input_stereo(OdometryObject* self, PyObject* const* args, Py_ssize_t nargs) {
    if (nargs != 3) {
        PyErr_SetString(PyExc_TypeError, "Invalid number of arguments. Expected: [left, right, timestamp]");
        return NULL;
    }
    PyObject* img_left = args[0];
    PyObject* img_right = args[1];
    double t = PyFloat_AsDouble(args[2]);
    cv::Mat mat_left = pbcvt::fromNDArrayToMat(img_left);
    cv::Mat mat_right = pbcvt::fromNDArrayToMat(img_right);
    cv::Mat mat_depth;
    int res = track_frame(self, t, mat_left, mat_right, mat_depth);
    return PyLong_FromLong(res);
}

static PyObject*
Odometry_input_depth(OdometryObject* self, PyObject* const* args, Py_ssize_t nargs) {
    if (nargs != 3) {
        PyErr_SetString(PyExc_TypeError, "Invalid number of arguments. Expected: [rgb, depth, timestamp]");
        return NULL;
    }
    PyObject* img_rgb = args[0];
    PyObject* img_depth = args[1];
    double t = PyFloat_AsDouble(args[2]);
    cv::Mat mat_left = pbcvt::fromNDArrayToMat(img_rgb);
    cv::Mat mat_right;
    cv::Mat mat_depth = pbcvt::fromNDArrayToMat(img_depth);
    int res = track_frame(self, t, mat_left, mat_right, mat_depth);
    return PyLong_FromLong(res);
}

static PyMethodDef Odometry_methods[] = {
    {"input_gyro", (PyCFunction) Odometry_input_gyro, METH_FASTCALL,
            PyDoc_STR("Add a gyro measurement.")},
    {"input_monocular", (PyCFunction) Odometry_input_monocular, METH_FASTCALL,
            PyDoc_STR("Add a monocular camera observation.")},
    {"input_stereo", (PyCFunction) Odometry_input_stereo, METH_FASTCALL,
            PyDoc_STR("Add a stereo camera observation.")},
    {"input_depth", (PyCFunction) Odometry_input_depth, METH_FASTCALL,
            PyDoc_STR("Add a depth camera observation.")},
    {NULL}  /* Sentinel */
};

static PyObject*
Odometry_get_pose(OdometryObject* self, void* _closure) {
    Eigen::Matrix<double, 3, 4> transform = self->refs->motion.T_last_.matrix3x4();
    double data[12] = {
        // NOTE: Klampt convention is column major.
        transform(0, 0), transform(1, 0), transform(2, 0),
        transform(0, 1), transform(1, 1), transform(2, 1),
        transform(0, 2), transform(1, 2), transform(2, 2),

        transform(0, 3), transform(1, 3), transform(2, 3)
    };
    return return_py_se3(data);
}

static int
Odometry_set_pose(PyObject* _self, PyObject* val, void* _closure) {
    OdometryObject* self = (OdometryObject*) _self;
    // NOTE: Doesn't override odometry. Only sets transform for motion model...
    // Can be used for initialization.
    double data[12];
    if (parse_py_se3(data, val)) { return -1; }
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot(data);
    Eigen::Map<Eigen::Vector3d> trans(data+9);
    self->refs->motion.T_last_ = Sophus::SE3d(rot.transpose(), trans);
    self->integrate_rot = self->refs->motion.T_last_.so3();
    return 0;
}

static PyMemberDef Odometry_members[] = {
    {NULL}  /* Sentinel */
};

static PyGetSetDef Odometry_getsetters[] = {
    {"pose", (getter) Odometry_get_pose, (setter) Odometry_set_pose,
    "Current estimated pose", NULL},
    {NULL}  /* Sentinel */
};

static PyTypeObject OdometryType = {
    .ob_base = PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "dsol.Odometry",
    .tp_basicsize = sizeof(OdometryObject),
    .tp_itemsize = 0,
    .tp_dealloc = (destructor) Odometry_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("DSOL Odometry module."),
    .tp_methods = Odometry_methods,
    .tp_members = Odometry_members,
    .tp_getset = Odometry_getsetters,
    .tp_init = (initproc) Odometry_init,
    .tp_new = (newfunc) Odometry_new,
};

static PyMethodDef DSOL_methods[] = {
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef DSOL_module = {
    PyModuleDef_HEAD_INIT,
    "dsol",                                 /* name of module */
    "Python bindings for DSOL odometry",    /* module documentation, may be NULL */
    -1,          /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    DSOL_methods
};


PyMODINIT_FUNC
PyInit_dsol(void) {
    PyObject *m;
    if (PyType_Ready(&OdometryType) < 0)
        return NULL;

    m = PyModule_Create(&DSOL_module);
    if (m == NULL)
        return NULL;

    Py_INCREF(&OdometryType);
    if (PyModule_AddObject(m, "Odometry", (PyObject *) &OdometryType) < 0) {
        Py_DECREF(&OdometryType);
        Py_DECREF(m);
        return NULL;
    }
    import_array();

    return m;
}
