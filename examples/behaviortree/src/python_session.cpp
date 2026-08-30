#include "python_session.hpp"

#include <stdexcept>

namespace
{
std::once_flag python_init_flag;

void initializePython()
{
  Py_Initialize();
  PyEval_SaveThread();
}
}  // namespace

PythonSession::PythonSession(const std::string& factory, const std::string& options)
{
  std::call_once(python_init_flag, initializePython);
  const PyGILState_STATE gil = PyGILState_Ensure();
  PyObject* module = PyImport_ImportModule("agimus_spacelab.tasks.task_planning.host");
  if(!module)
  {
    const auto error = pythonError();
    PyGILState_Release(gil);
    throw std::runtime_error(error);
  }
  PyObject* callable = PyObject_GetAttrString(module, factory.c_str());
  Py_DECREF(module);
  if(!callable || !PyCallable_Check(callable))
  {
    Py_XDECREF(callable);
    PyGILState_Release(gil);
    throw std::runtime_error("unknown allowlisted session factory: " + factory);
  }
  session_ = PyObject_CallFunction(callable, "s", options.c_str());
  Py_DECREF(callable);
  if(!session_)
  {
    const auto error = pythonError();
    PyGILState_Release(gil);
    throw std::runtime_error(error);
  }
  PyGILState_Release(gil);
}

PythonSession::~PythonSession()
{
  if(session_)
  {
    const PyGILState_STATE gil = PyGILState_Ensure();
    Py_DECREF(session_);
    PyGILState_Release(gil);
  }
}

std::string PythonSession::call(const std::string& method)
{
  return callImpl(method, nullptr);
}

std::string PythonSession::call(const std::string& method, const std::string& argument)
{
  return callImpl(method, &argument);
}

std::string PythonSession::callImpl(const std::string& method,
                                    const std::string* argument)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const PyGILState_STATE gil = PyGILState_Ensure();
  PyObject* callable = PyObject_GetAttrString(session_, method.c_str());
  if(!callable || !PyCallable_Check(callable))
  {
    Py_XDECREF(callable);
    const auto error = pythonError();
    PyGILState_Release(gil);
    throw std::runtime_error(error);
  }
  PyObject* result = argument ? PyObject_CallFunction(callable, "s", argument->c_str())
                              : PyObject_CallNoArgs(callable);
  Py_DECREF(callable);
  if(!result)
  {
    const auto error = pythonError();
    PyGILState_Release(gil);
    throw std::runtime_error(error);
  }
  const char* text = PyUnicode_AsUTF8(result);
  if(!text)
  {
    Py_DECREF(result);
    const auto error = pythonError();
    PyGILState_Release(gil);
    throw std::runtime_error(error);
  }
  std::string output(text);
  Py_DECREF(result);
  PyGILState_Release(gil);
  return output;
}

std::string PythonSession::pythonError()
{
  std::string output = "Python call failed";
  PyObject* error_type = nullptr;
  PyObject* error_value = nullptr;
  PyObject* error_traceback = nullptr;
  PyErr_Fetch(&error_type, &error_value, &error_traceback);
  PyErr_NormalizeException(&error_type, &error_value, &error_traceback);
  if(error_value)
  {
    PyObject* text_object = PyObject_Str(error_value);
    if(text_object)
    {
      if(const char* text = PyUnicode_AsUTF8(text_object))
      {
        output = text;
      }
      Py_DECREF(text_object);
    }
  }
  Py_XDECREF(error_type);
  Py_XDECREF(error_value);
  Py_XDECREF(error_traceback);
  return output;
}