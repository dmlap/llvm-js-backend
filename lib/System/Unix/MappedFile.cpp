//===- Unix/MappedFile.cpp - Unix MappedFile Implementation -----*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file was developed by Reid Spencer and is distributed under the 
// University of Illinois Open Source License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file provides the generic Unix implementation of the MappedFile concept.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//=== WARNING: Implementation here must contain only generic UNIX code that
//===          is guaranteed to work on *all* UNIX variants.
//===----------------------------------------------------------------------===//

#include "llvm/System/Process.h"
#include "Unix.h"
#include <fcntl.h>
#include <sys/mman.h>

namespace llvm {
using namespace sys;

struct sys::MappedFileInfo {
  int fd_;
  struct stat sbuf_;
};

void MappedFile::initialize() {
  if (path_.exists()) {
    info_ = new MappedFileInfo;
    int mode = 0;
    if (options_&READ_ACCESS) 
      if (options_&WRITE_ACCESS)
        mode = O_RDWR;
      else
        mode = O_RDONLY;
    else if (options_&WRITE_ACCESS)
      mode = O_WRONLY;
    info_->fd_ = ::open(path_.c_str(),mode);
    if (info_->fd_ < 0) {
      delete info_;
      info_ = 0;
      ThrowErrno(std::string("Can't open file: ") + path_.get());
    }
    struct stat sbuf;
    if(::fstat(info_->fd_, &info_->sbuf_) < 0) {
      ::close(info_->fd_);
      delete info_;
      info_ = 0;
      ThrowErrno(std::string("Can't stat file: ") + path_.get());
    }
  } else {
    throw std::string("Can't open file: ") + path_.get();
  }
}

void MappedFile::terminate() {
  assert(info_ && "MappedFile not initialized");
  if (info_->fd_ >= 0)
    ::close(info_->fd_);
  delete info_;
  info_ = 0;
}

void MappedFile::unmap() {
  assert(info_ && "MappedFile not initialized");
  if (isMapped()) {
    if (options_ & WRITE_ACCESS)
      ::msync(base_, info_->sbuf_.st_size, MS_SYNC);
    ::munmap(base_, info_->sbuf_.st_size);
  }
}

void* MappedFile::map() {
  assert(info_ && "MappedFile not initialized");
  if (!isMapped()) {
    int prot = PROT_NONE;
    int flags = 0;
#ifdef MAP_FILE
    flags |= MAP_FILE;
#endif
    if (options_ == 0) {
      prot = PROT_READ;
      flags = MAP_PRIVATE;
    } else {
      if (options_ & READ_ACCESS)
        prot |= PROT_READ;
      if (options_ & WRITE_ACCESS)
        prot |= PROT_WRITE;
      if (options_ & EXEC_ACCESS)
        prot |= PROT_EXEC;
      if (options_ & SHARED_MAPPING)
        flags |= MAP_SHARED;
      else
        flags |= MAP_PRIVATE;
    }
    size_t map_size = ((info_->sbuf_.st_size / Process::GetPageSize())+1) *
      Process::GetPageSize();

    base_ = ::mmap(0, map_size, prot, flags, info_->fd_, 0);
    if (base_ == MAP_FAILED)
      ThrowErrno(std::string("Can't map file:") + path_.get());
  }
  return base_;
}

size_t MappedFile::size() {
  assert(info_ && "MappedFile not initialized");
  return info_->sbuf_.st_size;
}

void MappedFile::size(size_t new_size) {
  assert(info_ && "MappedFile not initialized");

  // Take the mapping out of memory
  this->unmap();

  // Adjust the current size to a page boundary
  size_t cur_size = ((info_->sbuf_.st_size / Process::GetPageSize())+1) *
    Process::GetPageSize();

  // Adjust the new_size to a page boundary
  new_size = ((new_size / Process::GetPageSize())+1) *
    Process::GetPageSize();

  // If the file needs to be extended
  if (new_size > cur_size) {
    // Ensure we can allocate at least the idodes necessary to handle the
    // file size requested. 
    ::lseek(info_->fd_, new_size, SEEK_SET);
    ::write(info_->fd_, "\0", 1);
  }

  // Seek to current end of file. 
  this->map();
}

}

// vim: sw=2 smartindent smarttab tw=80 autoindent expandtab
