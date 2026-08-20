#include "PythonEngine.hpp"
#include <iostream>
#include <vector>
#include <pthread.h>
#include <sched.h>
#include "src/npu/ProtocolSerializer.hpp"

namespace
{
    // Global function pointer references to be called by CPython bindings
    void (*g_apmControllerARM)(void) = nullptr;
    void (*g_apmControllerDISARM)(void) = nullptr;
    void (*g_apmControllerPosition)(int, int, int, bool) = nullptr;
    void (*g_apmControllerSpeed)(int, int, int) = nullptr;
    void (*g_apmControllerServo)(int, int) = nullptr;
    void (*g_pushBroadcastData)(std::vector<uint8_t>) = nullptr;

    template <typename T>
    inline bool isValidPointer(T *ptr)
    {
        if (ptr == nullptr)
            return false;
        uintptr_t val = reinterpret_cast<uintptr_t>(ptr);
        
        // 1. Check for standard invalid/sentinel values (-1)
        if (val == static_cast<uintptr_t>(-1) || val == 0xffffffff)
            return false;
            
        // 2. Filter out typical uninitialized memory filler patterns
        if (val == 0xbebebebebebebebeULL || val == 0xbebebebeULL)
            return false;
        if (val == 0xccccccccccccccccULL || val == 0xccccccccULL)
            return false;
        if (val == 0xcdcdcdcdcdcdcdcdULL || val == 0xcdcdcdcdULL)
            return false;
        if (val == 0xddddddddddddddddULL || val == 0xddddddddULL)
            return false;
        if (val == 0xdeadbeefdeadbeefULL || val == 0xdeadbeefULL)
            return false;
            
        // 3. User-space virtual address range check on 64-bit Linux.
        // Valid user-space addresses are less than 0x0001000000000000ULL (48-bit address space).
        // Addresses below 0x1000 are reserved/invalid.
        if (val < 0x1000 || val >= 0x0001000000000000ULL)
            return false;
            
        return true;
    }
}

// CPython wrappers implementation
PyObject *PythonEngine::apm_ARM(PyObject *self, PyObject *args)
{
    if (g_apmControllerARM)
    {
        g_apmControllerARM();
        Py_RETURN_NONE;
    }
    PyErr_SetString(PyExc_RuntimeError, "APMControllerARM pointer is null");
    return nullptr;
}

PyObject *PythonEngine::apm_DISARM(PyObject *self, PyObject *args)
{
    if (g_apmControllerDISARM)
    {
        g_apmControllerDISARM();
        Py_RETURN_NONE;
    }
    PyErr_SetString(PyExc_RuntimeError, "APMControllerDISARM pointer is null");
    return nullptr;
}

PyObject *PythonEngine::apm_Position(PyObject *self, PyObject *args)
{
    int x, y, z;
    PyObject *resetHomeObj;
    if (!PyArg_ParseTuple(args, "iiiO", &x, &y, &z, &resetHomeObj))
    {
        return nullptr;
    }
    bool resetHome = PyObject_IsTrue(resetHomeObj);
    if (g_apmControllerPosition)
    {
        g_apmControllerPosition(x, y, z, resetHome);
        Py_RETURN_NONE;
    }
    PyErr_SetString(PyExc_RuntimeError, "APMControllerPosition pointer is null");
    return nullptr;
}

PyObject *PythonEngine::apm_Speed(PyObject *self, PyObject *args)
{
    int x, y, z;
    if (!PyArg_ParseTuple(args, "iii", &x, &y, &z))
    {
        return nullptr;
    }
    if (g_apmControllerSpeed)
    {
        g_apmControllerSpeed(x, y, z);
        Py_RETURN_NONE;
    }
    PyErr_SetString(PyExc_RuntimeError, "APMControllerSpeed pointer is null");
    return nullptr;
}

PyObject *PythonEngine::apm_Servo(PyObject *self, PyObject *args)
{
    int pin, pwm;
    if (!PyArg_ParseTuple(args, "ii", &pin, &pwm))
    {
        return nullptr;
    }
    if (g_apmControllerServo)
    {
        g_apmControllerServo(pin, pwm);
        Py_RETURN_NONE;
    }
    PyErr_SetString(PyExc_RuntimeError, "APMControllerServo pointer is null");
    return nullptr;
}

PyObject *PythonEngine::apm_PushBroadcast(PyObject *self, PyObject *args)
{
    Py_buffer view;
    if (!PyArg_ParseTuple(args, "y*", &view))
    {
        return nullptr;
    }
    if (g_pushBroadcastData)
    {
        std::vector<uint8_t> vec((uint8_t *)view.buf, (uint8_t *)view.buf + view.len);
        g_pushBroadcastData(vec);
        PyBuffer_Release(&view);
        Py_RETURN_NONE;
    }
    PyBuffer_Release(&view);
    PyErr_SetString(PyExc_RuntimeError, "pushBroadcastData pointer is null");
    return nullptr;
}

static PyMethodDef APMMethods[] = {
    {"arm", PythonEngine::apm_ARM, METH_NOARGS, "Arm the flight controller"},
    {"disarm", PythonEngine::apm_DISARM, METH_NOARGS, "Disarm the flight controller"},
    {"set_position", PythonEngine::apm_Position, METH_VARARGS, "Set flight controller target position (x, y, z, resetHome)"},
    {"set_speed", PythonEngine::apm_Speed, METH_VARARGS, "Set flight controller speed (x, y, z)"},
    {"set_servo", PythonEngine::apm_Servo, METH_VARARGS, "Set servo output (pin, pwm)"},
    {"push_broadcast", PythonEngine::apm_PushBroadcast, METH_VARARGS, "Push broadcast data"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef apmmodule = {
    PyModuleDef_HEAD_INIT,
    "apm",
    "APM Flight Controller Module",
    -1,
    APMMethods};

PyMODINIT_FUNC PyInit_apm(void)
{
    return PyModule_Create(&apmmodule);
}

PythonEngine::PythonEngine()
    : m_pythonModule(nullptr), g_pythonInitFunc(nullptr), g_pythonExchangeFunc(nullptr), m_mainThreadState(nullptr), m_initialized(false), m_affinitySet(false)
{
}

PythonEngine::~PythonEngine()
{
    cleanup();
}

bool PythonEngine::initialize(const V4L2Tools::V4l2Info &vinfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_initialized)
        return true;

    std::cout << "[PythonEngine] Registering APM module & Initializing Python..." << std::endl;
    PyImport_AppendInittab("apm", PyInit_apm);
    Py_Initialize();

    PyRun_SimpleString("import sys; sys.path.append('/etc/rknn')");

    m_pythonModule = PyImport_ImportModule("user_app");
    if (m_pythonModule != nullptr)
    {
        g_pythonInitFunc = PyObject_GetAttrString(m_pythonModule, "init");
        g_pythonExchangeFunc = PyObject_GetAttrString(m_pythonModule, "exchange");

        if (g_pythonInitFunc && PyCallable_Check(g_pythonInitFunc))
        {
            std::cout << "[PythonEngine] Calling Python init()..." << std::endl;
            PyObject *pArgs = Py_BuildValue("(iii)", vinfo.ImgWidth, vinfo.ImgHeight, vinfo.PixFormat);
            PyObject *pValue = PyObject_CallObject(g_pythonInitFunc, pArgs);
            Py_XDECREF(pArgs);
            Py_XDECREF(pValue);
        }
        m_initialized = true;
        m_mainThreadState = PyEval_SaveThread(); // Release GIL
        return true;
    }
    else
    {
        PyErr_Print();
        std::cerr << "[PythonEngine] Failed to import Python module 'user_app'" << std::endl;
        return false;
    }
}

void PythonEngine::cleanup()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized)
        return;

    // Use PyGILState to safely decrement reference counts on any thread
    PyGILState_STATE gstate = PyGILState_Ensure();

    Py_XDECREF(g_pythonInitFunc);
    Py_XDECREF(g_pythonExchangeFunc);
    Py_XDECREF(m_pythonModule);

    g_pythonInitFunc = nullptr;
    g_pythonExchangeFunc = nullptr;
    m_pythonModule = nullptr;

    PyGILState_Release(gstate);

    std::cout << "[PythonEngine] Cleaned up Python references." << std::endl;
    m_initialized = false;
}

PyObject *PythonEngine::packFloatArray(float *const *arr, int size)
{
    PyObject *pList = PyList_New(size);
    for (int i = 0; i < size; ++i)
    {
        if (isValidPointer(arr[i]))
        {
            PyObject *val = PyFloat_FromDouble(*arr[i]);
            PyList_SetItem(pList, i, val);
        }
        else
        {
            Py_INCREF(Py_None);
            PyList_SetItem(pList, i, Py_None);
        }
    }
    return pList;
}

PyObject *PythonEngine::packIntArray(int *const *arr, int size)
{
    PyObject *pList = PyList_New(size);
    for (int i = 0; i < size; ++i)
    {
        if (isValidPointer(arr[i]))
        {
            PyObject *val = PyLong_FromLong(*arr[i]);
            PyList_SetItem(pList, i, val);
        }
        else
        {
            Py_INCREF(Py_None);
            PyList_SetItem(pList, i, Py_None);
        }
    }
    return pList;
}

PyObject *PythonEngine::packDoubleArray(double *const *arr, int size)
{
    PyObject *pList = PyList_New(size);
    for (int i = 0; i < size; ++i)
    {
        if (isValidPointer(arr[i]))
        {
            PyObject *val = PyFloat_FromDouble(*arr[i]);
            PyList_SetItem(pList, i, val);
        }
        else
        {
            Py_INCREF(Py_None);
            PyList_SetItem(pList, i, Py_None);
        }
    }
    return pList;
}

PyObject *PythonEngine::buildTelemetryDict(const ControllerData &apmData, const std::vector<std::vector<uint8_t>> &broadcastPackets)
{
    PyObject *pTelemetry = PyDict_New();

    auto add_float = [&](const char *key, float *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyFloat_FromDouble(*ptr);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_double = [&](const char *key, double *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyFloat_FromDouble(*ptr);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_int = [&](const char *key, int *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyLong_FromLong(*ptr);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_uint16 = [&](const char *key, uint16_t *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyLong_FromLong(*ptr);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_uint64 = [&](const char *key, uint64_t *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyLong_FromUnsignedLongLong(*ptr);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_bool = [&](const char *key, bool *ptr)
    {
        if (isValidPointer(ptr))
        {
            PyObject *val = PyBool_FromLong(*ptr ? 1 : 0);
            PyDict_SetItemString(pTelemetry, key, val);
            Py_DECREF(val);
        }
    };
    auto add_array = [&](const char *key, PyObject *listObj)
    {
        if (listObj)
        {
            PyDict_SetItemString(pTelemetry, key, listObj);
            Py_DECREF(listObj);
        }
    };

    // Populate values
    add_int("accel_clipped_times", apmData._Accel_ClippedTimes);
    add_float("accel_gforce", apmData._Accel_GForce);
    add_float("baro_temp", apmData._Baro_Temp);
    add_float("baro_pressure_hpa", apmData._Baro_PressureHPA);
    add_float("baro_agl_altitude_cm", apmData._Baro_AGLAltitudeCM);
    add_double("rangefinder_agl_alt_cm", apmData._RangeFinder_AGLAltCM);
    add_bool("sys_arm_flag", apmData._SYS_ARMFlag);
    add_uint16("sys_pre_arm_flag", apmData._SYS_PreARMFlag);
    add_uint16("sys_failsafe_flag", apmData._SYS_FailSafeFlag);
    add_int("sys_apm_status", apmData._SYS_APMStatus);
    add_float("nav_relative_head", apmData._NAV_Relative_Head);
    add_float("nav_global_head", apmData._NAV_Global_Head);
    add_int("nav_global_sat_count", apmData._NAV_Global_SATCount);
    add_int("nav_gps_hdop", apmData._NAV_GPS_HDOP);
    add_uint64("gyro_cycle_time", apmData._GYRO_CYCLE_TIME);
    add_float("battery_voltage", apmData._Battery_Voltage);
    add_float("battery_voltage_single", apmData._Battery_Voltage_Single);
    add_double("cpu_temp", apmData._CPU_Core_Temp);

    // Arrays
    add_array("accel_acceleration", packFloatArray(apmData._Accel_Accelration, 3));
    add_array("accel_vibe", packFloatArray(apmData._Accel_VIBE, 3));
    add_array("accel_raw_g", packIntArray(apmData._Accel_RawG, 3));
    add_array("att_quaternion", packFloatArray(apmData._ATT_Quaterion, 4));
    add_array("att_euler_angle", packFloatArray(apmData._ATT_EulerAngle, 3));
    add_array("gyro_angle_rate", packFloatArray(apmData._Gyro_AngleRate, 3));
    add_array("mag_raw_l", packIntArray(apmData._Mag_RawL, 3));
    add_array("sys_time_info", packIntArray(apmData._SYS_TimeInfo, 10));
    add_array("nav_speed", packDoubleArray(apmData._NAV_Speed, 3));
    add_array("nav_global_speed", packDoubleArray(apmData._NAV_Global_Speed, 2));
    add_array("nav_global_pos", packIntArray(apmData._NAV_Global_Pos, 3));
    add_array("nav_global_home", packIntArray(apmData._NAV_Global_HOME, 2));
    add_array("nav_relative_pos", packDoubleArray(apmData._NAV_Relative_Pos, 3));
    add_array("rc_channel_raw", packIntArray(apmData._RC_Channel_Raw, 16));
    add_array("ef_channel_raw", packIntArray(apmData._EF_Channel_Raw, 16));

    // Broadcast received packets
    PyObject *pList = PyList_New(broadcastPackets.size());
    for (size_t i = 0; i < broadcastPackets.size(); ++i)
    {
        PyObject *pBytes = nullptr;
        if (broadcastPackets[i].empty())
        {
            pBytes = PyBytes_FromStringAndSize("", 0);
        }
        else
        {
            pBytes = PyBytes_FromStringAndSize((const char *)broadcastPackets[i].data(), broadcastPackets[i].size());
        }
        PyList_SetItem(pList, i, pBytes);
    }
    PyDict_SetItemString(pTelemetry, "BroadcastRecv", pList);
    Py_DECREF(pList);

    // Pack latest NPU detections
    std::vector<TrackedBox> latestDets = ProtocolSerializer::getLatestDetections();
    PyObject *pDetsList = PyList_New(latestDets.size());
    for (size_t i = 0; i < latestDets.size(); ++i)
    {
        PyObject *pDetDict = PyDict_New();
        
        PyObject *pTrackId = PyLong_FromLong(latestDets[i].track_id);
        PyObject *pClassId = PyLong_FromLong(latestDets[i].class_id);
        PyObject *pConf = PyFloat_FromDouble(latestDets[i].confidence);
        
        PyObject *pBox = PyList_New(4);
        PyList_SetItem(pBox, 0, PyLong_FromLong(latestDets[i].x1));
        PyList_SetItem(pBox, 1, PyLong_FromLong(latestDets[i].y1));
        PyList_SetItem(pBox, 2, PyLong_FromLong(latestDets[i].x2));
        PyList_SetItem(pBox, 3, PyLong_FromLong(latestDets[i].y2));
        
        PyDict_SetItemString(pDetDict, "track_id", pTrackId);
        PyDict_SetItemString(pDetDict, "class_id", pClassId);
        PyDict_SetItemString(pDetDict, "confidence", pConf);
        PyDict_SetItemString(pDetDict, "box", pBox);
        
        Py_DECREF(pTrackId);
        Py_DECREF(pClassId);
        Py_DECREF(pConf);
        Py_DECREF(pBox);
        
        PyList_SetItem(pDetsList, i, pDetDict);
    }
    PyDict_SetItemString(pTelemetry, "detections", pDetsList);
    Py_DECREF(pDetsList);

    return pTelemetry;
}

bool PythonEngine::execute(const UserAppData &data, const std::vector<std::vector<uint8_t>> &broadcastPackets)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized || !g_pythonExchangeFunc || !PyCallable_Check(g_pythonExchangeFunc))
    {
        return false;
    }

    if (!m_affinitySet)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset); // Bind to CPU 2
        pthread_t current_thread = pthread_self();
        int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
        if (rc == 0)
        {
            std::cout << "[PythonEngine] Successfully bound Python execution thread to CPU 2" << std::endl;
        }
        else
        {
            std::cerr << "[PythonEngine] Failed to bind Python execution thread to CPU 2, error: " << rc << std::endl;
        }
        m_affinitySet = true;
    }

    // Set callback pointers globally
    g_apmControllerARM = data.APMData.APMControllerARM;
    g_apmControllerDISARM = data.APMData.APMControllerDISARM;
    g_apmControllerPosition = data.APMData.APMControllerPosition;
    g_apmControllerSpeed = data.APMData.APMControllerSpeed;
    g_apmControllerServo = data.APMData.APMControllerServo;
    g_pushBroadcastData = data.pushBroadcastData;

    // Ensure we hold the GIL when calling Python C APIs
    PyGILState_STATE gstate = PyGILState_Ensure();

    // Pack telemetry and camera frame data
    PyObject *pTelemetry = buildTelemetryDict(data.APMData, broadcastPackets);
    PyObject *pFrameBytes = PyBytes_FromStringAndSize((const char *)data.cameraFrame.data, data.cameraFrame.size);

    // Pass args (frame_bytes, width, height, pixfmt, telemetry)
    PyObject *pArgs = Py_BuildValue("(OiiiO)", pFrameBytes, data.cameraFrame.width, data.cameraFrame.height, data.cameraFrame.pixfmt, pTelemetry);

    PyObject *pValue = PyObject_CallObject(g_pythonExchangeFunc, pArgs);

    Py_XDECREF(pFrameBytes);
    Py_XDECREF(pTelemetry);
    Py_XDECREF(pArgs);
    Py_XDECREF(pValue);

    PyGILState_Release(gstate);

    return (pValue != nullptr);
}
