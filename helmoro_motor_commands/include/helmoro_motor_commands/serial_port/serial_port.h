#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>

class SerialPort {
  public:
    // constructor & destructor
    SerialPort(void);
    ~SerialPort(void);

    // public set up & get functions
    bool Initialize(const std::string port, const int baud);
    int GetFileDescriptor(void) const { return fd_; };

    // public read/write functions
    int Write(const void* buffer, const unsigned int length);
    int Read(void* buffer, const unsigned int bufferLength);
    void Flush(int dir);

  private:
    // general variables
    int fd_;
};