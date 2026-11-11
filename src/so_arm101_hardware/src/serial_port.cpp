// Copyright 2026 so_arm101 authors. MIT License.
#include <algorithm>
#include "so_arm101_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace so_arm101_hardware
{

namespace
{
bool to_speed(int baud, speed_t * out)
{
  switch (baud) {
    case 115200: *out = B115200; return true;
    case 230400: *out = B230400; return true;
    case 460800: *out = B460800; return true;
    case 500000: *out = B500000; return true;
    case 576000: *out = B576000; return true;
    case 921600: *out = B921600; return true;
    case 1000000: *out = B1000000; return true;
    default: return false;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & path, int baud, std::string * err)
{
  close();
  speed_t speed;
  if (!to_speed(baud, &speed)) {
    if (err) {*err = "unsupported baudrate " + std::to_string(baud);}
    return false;
  }
  int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    if (err) {*err = "open(" + path + ") failed: " + std::strerror(errno);}
    return false;
  }
  if (::ioctl(fd, TIOCEXCL) != 0) {
    if (err) {*err = "TIOCEXCL failed (port in use?): " + std::string(std::strerror(errno));}
    ::close(fd);
    return false;
  }
  termios tio;
  if (::tcgetattr(fd, &tio) != 0) {
    if (err) {*err = "tcgetattr failed: " + std::string(std::strerror(errno));}
    ::close(fd);
    return false;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);  // 8N1、フロー制御なし
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    if (err) {*err = "tcsetattr failed: " + std::string(std::strerror(errno));}
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write_all(const uint8_t * data, size_t len)
{
  if (fd_ < 0) {return false;}
  size_t done = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (done < len) {
    ssize_t n = ::write(fd_, data + done, len - done);
    if (n > 0) {
      done += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      if (std::chrono::steady_clock::now() > deadline) {return false;}
      pollfd p{fd_, POLLOUT, 0};
      ::poll(&p, 1, 5);
      continue;
    }
    return false;
  }
  return true;
}

size_t SerialPort::read_exact(uint8_t * buf, size_t n, std::chrono::microseconds timeout)
{
  if (fd_ < 0) {return 0;}
  size_t got = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (got < n) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {break;}
    auto remain_ms = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    pollfd p{fd_, POLLIN, 0};
    int rc = ::poll(&p, 1, static_cast<int>((remain_ms + 999) / 1000));
    if (rc < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    if (rc == 0) {break;}
    ssize_t r = ::read(fd_, buf + got, n - got);
    if (r > 0) {
      got += static_cast<size_t>(r);
    } else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EINTR)) {
      break;
    }
  }
  return got;
}

size_t SerialPort::flush_input()
{
  if (fd_ < 0) {return 0;}
  int pending = 0;
  if (::ioctl(fd_, FIONREAD, &pending) != 0 || pending < 0) {pending = 0;}
  ::tcflush(fd_, TCIFLUSH);
  return static_cast<size_t>(pending);
}

size_t SerialPort::drain_until_quiet(std::chrono::microseconds quiet, std::chrono::microseconds max_total)
{
  if (fd_ < 0) {return 0;}
  size_t total = 0;
  const auto deadline = std::chrono::steady_clock::now() + max_total;
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {break;}
    const auto wait = std::min(quiet, std::chrono::duration_cast<std::chrono::microseconds>(deadline - now));
    timespec ts{};
    ts.tv_sec = wait.count() / 1000000;
    ts.tv_nsec = (wait.count() % 1000000) * 1000;
    pollfd p{fd_, POLLIN, 0};
    const int rc = ::ppoll(&p, 1, &ts, nullptr);
    if (rc < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    if (rc == 0) {break;}  // quiet の間、何も来なかった
    uint8_t tmp[256];
    const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) {
      total += static_cast<size_t>(n);
    } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
      break;
    }
  }
  ::tcflush(fd_, TCIFLUSH);
  return total;
}

}  // namespace so_arm101_hardware
