#include "PythonRunner.hpp"

#include <RobWorkStudioConfig.hpp>

#include <iostream>

#ifdef _GNU_SOURCE
#define _POSIX_C_SOURCE_OLD _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE

#define _XOPEN_SOURCE_OLD _XOPEN_SOURCE
#undef _XOPEN_SOURCE

#include <Python.h>

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE _POSIX_C_SOURCE_OLD

#undef _XOPEN_SOURCE
#define _XOPEN_SOURCE _XOPEN_SOURCE_OLD
#endif
#ifndef _GNU_SOURCE
#include <Python.h>
#endif

using namespace rws::python;

/*
std::ostream& operator<< (std::ostream& os, PyThreadState* ts)
{
    os << "PyThreadState {" << std::endl;
    os << "    pointer: " << size_t (ts) << std::endl;
    if (ts) {
        os << "    id: " << ts->id << std::endl;
        os << "    GIL_count: " << ts->gilstate_counter << std::endl;
        os << "    prev_id: " << size_t (ts->prev) << std::endl;
        os << "    next_id: " << size_t (ts->next) << std::endl;
    }
    os << "}";
    return os;
}
*/
PyThreadState* ENV_main_thread;
// swap the current thread state with ts, restore when the object goes out of scope

PythonLock::PythonLock (PyThreadState* ts)
{
    //swap interpretor
    _threadState = PyThreadState_Swap (ts);

    //Aquire Lock
    PyGILState_Ensure ();
}

PythonLock::~PythonLock ()
{
    //Release Lock
    PyGILState_Release (PyGILState_STATE::PyGILState_UNLOCKED);

    //Switch back to main thread
    PyThreadState_Swap (_threadState);
}

PythonRunner::PythonRunner () : _threadState (NULL)
{
    initPython ();
    PyGILState_Ensure ();
    _threadState = Py_NewInterpreter ();

    PyThreadState_Swap (ENV_main_thread);
    PyGILState_Release (PyGILState_STATE::PyGILState_UNLOCKED);
}

PythonRunner::~PythonRunner ()
{
    if (_threadState) {
        PyThreadState* ts = PyThreadState_Swap (_threadState);;
        Py_EndInterpreter (_threadState);

        PyThreadState_Swap(ts);
    }
}

int PythonRunner::runCode (std::string code)
{
    PythonLock swap (_threadState);

    int ret = PyRun_SimpleString (code.c_str ());
    return ret;
}

void PythonRunner::initPython ()
{
    // Initialize Python
    if (!Py_IsInitialized ()) {
#ifdef RWS_USE_PYTHON3
        wchar_t* program = Py_DecodeLocale ("RobWorkStudio Python Interpretor", NULL);
        if (program == NULL) {
            return;
        }
#endif
#ifdef RWS_USE_PYTHON2
        char program[] = "RobWorkStudio Python Interpretor";
#endif
        Py_SetProgramName (program);
        Py_InitializeEx (1);
#if PYTHON_VERSION_MINOR < 9 && defined(RWS_USE_PYTHON3)
        PyEval_InitThreads ();
#endif 
        ENV_main_thread = PyThreadState_Get ();
    }
}