#pragma once

#include <Python.h>
#include <string>
#include <mutex>
#include "Public/PLGUserDefine.hpp"

class PythonEngine
{
public:
    PythonEngine();
    ~PythonEngine();

    bool initialize(const V4L2Tools::V4l2Info &vinfo);
    void cleanup();

    bool execute(const UserAppData &data, const std::vector<std::vector<uint8_t>> &broadcastPackets);

    // CPython wrapper helper functions (must be public for static linkage inside APMMethods array)
    static PyObject* apm_ARM(PyObject* self, PyObject* args);
    static PyObject* apm_DISARM(PyObject* self, PyObject* args);
    static PyObject* apm_Position(PyObject* self, PyObject* args);
    static PyObject* apm_Speed(PyObject* self, PyObject* args);
    static PyObject* apm_Servo(PyObject* self, PyObject* args);
    static PyObject* apm_PushBroadcast(PyObject* self, PyObject* args);

private:

    static PyObject* packFloatArray(float* const* arr, int size);
    static PyObject* packIntArray(int* const* arr, int size);
    static PyObject* packDoubleArray(double* const* arr, int size);

    PyObject* buildTelemetryDict(const ControllerData &apmData, const std::vector<std::vector<uint8_t>> &broadcastPackets);

private:
    PyObject* m_pythonModule;
    PyObject* g_pythonInitFunc;
    PyObject* g_pythonExchangeFunc;
    PyThreadState* m_mainThreadState;
    mutable std::mutex m_mutex;
    bool m_initialized;
    bool m_affinitySet;
};
