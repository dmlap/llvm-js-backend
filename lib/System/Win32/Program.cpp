//===- Win32/Program.cpp - Win32 Program Implementation ------- -*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file was developed by Jeff Cohen and is distributed under the 
// University of Illinois Open Source License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file provides the Win32 specific implementation of the Program class.
//
//===----------------------------------------------------------------------===//

#include "Win32.h"
#include <malloc.h>

//===----------------------------------------------------------------------===//
//=== WARNING: Implementation here must contain only Win32 specific code 
//===          and must not be UNIX code
//===----------------------------------------------------------------------===//

namespace llvm {
using namespace sys;

// This function just uses the PATH environment variable to find the program.
Path
Program::FindProgramByName(const std::string& progName) {

  // Check some degenerate cases
  if (progName.length() == 0) // no program
    return Path();
  Path temp;
  if (!temp.setFile(progName)) // invalid name
    return Path();
  if (temp.executable()) // already executable as is
    return temp;

  // At this point, the file name is valid and its not executable.
  // Let Windows search for it.
  char buffer[MAX_PATH];
  char *dummy = NULL;
  DWORD len = SearchPath(NULL, progName.c_str(), ".exe", MAX_PATH,
                         buffer, &dummy);

  // See if it wasn't found.
  if (len == 0)
    return Path();

  // See if we got the entire path.
  if (len < MAX_PATH)
    return Path(buffer);

  // Buffer was too small; grow and retry.
  while (true) {
    char *b = reinterpret_cast<char *>(_alloca(len+1));
    DWORD len2 = SearchPath(NULL, progName.c_str(), ".exe", len+1, b, &dummy);

    // It is unlikely the search failed, but it's always possible some file
    // was added or removed since the last search, so be paranoid...
    if (len2 == 0)
      return Path();
    else if (len2 <= len)
      return Path(b);

    len = len2;
  }
}

//
int 
Program::ExecuteAndWait(const Path& path, 
                        const char** args,
                        const char** envp,
                        const Path** redirects,
                        unsigned secondsToWait) {
  if (!path.executable())
    throw path.toString() + " is not executable"; 

  // Windows wants a command line, not an array of args, to pass to the new
  // process.  We have to concatenate them all, while quoting the args that
  // have embedded spaces.

  // First, determine the length of the command line.
  std::string progname(path.getLast());
  unsigned len = progname.length() + 1;
  if (progname.find(' ') != std::string::npos)
    len += 2;

  for (unsigned i = 0; args[i]; i++) {
    len += strlen(args[i]) + 1;
    if (strchr(args[i], ' '))
      len += 2;
  }

  // Now build the command line.
  char *command = reinterpret_cast<char *>(_alloca(len));
  char *p = command;

  bool needsQuoting = progname.find(' ') != std::string::npos;
  if (needsQuoting)
    *p++ = '"';
  memcpy(p, progname.c_str(), progname.length());
  p += progname.length();
  if (needsQuoting)
    *p++ = '"';
  *p++ = ' ';

  for (unsigned i = 0; args[i]; i++) {
    const char *arg = args[i];
	size_t len = strlen(arg);
    needsQuoting = strchr(arg, ' ') != 0;
    if (needsQuoting)
      *p++ = '"';
    memcpy(p, arg, len);
    p += len;
    if (needsQuoting)
      *p++ = '"';
    *p++ = ' ';
  }

  *p = 0;

  // Create a child process.
  STARTUPINFO si;
  memset(&si, 0, sizeof(si));
  si.cb = sizeof(si);

  // TODO: do replacement of standard input/output/error handles.

  PROCESS_INFORMATION pi;
  memset(&pi, 0, sizeof(pi));

  if (!CreateProcess(path.c_str(), command, NULL, NULL, FALSE, 0,
                     envp, NULL, &si, &pi))
  {
    ThrowError(std::string("Couldn't execute program '") + 
               path.toString() + "'");
  }

  // Wait for it to terminate.
  DWORD millisecondsToWait = INFINITE;
  if (secondsToWait > 0)
    millisecondsToWait = secondsToWait * 1000;

  if (WaitForSingleObject(pi.hProcess, millisecondsToWait) == WAIT_TIMEOUT) {
    if (!TerminateProcess(pi.hProcess, 1)) {
      ThrowError(std::string("Failed to terminate timed-out program '") + 
                 path.toString() + "'");
    }
    WaitForSingleObject(pi.hProcess, INFINITE);
  }
  
  // Get its exit status.
  DWORD status;
  BOOL rc = GetExitCodeProcess(pi.hProcess, &status);

  // Done with the handles; go close them.
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);

  if (!rc)
    ThrowError(std::string("Failed getting status for program '") + 
               path.toString() + "'");

  return status;
}

}
// vim: sw=2 smartindent smarttab tw=80 autoindent expandtab
