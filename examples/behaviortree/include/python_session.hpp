#pragma once

#include <Python.h>

#include <mutex>
#include <string>

class PythonSession
{
public:
  explicit PythonSession(const std::string& factory, const std::string& options = "{}");
  ~PythonSession();

  PythonSession(const PythonSession&) = delete;
  PythonSession& operator=(const PythonSession&) = delete;

  std::string call(const std::string& method);
  std::string call(const std::string& method, const std::string& argument);

private:
  std::string callImpl(const std::string& method, const std::string* argument);
  static std::string pythonError();

  PyObject* session_ = nullptr;
  std::mutex mutex_;
};