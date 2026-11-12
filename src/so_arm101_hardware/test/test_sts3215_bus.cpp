// Copyright 2026 so_arm101 authors. MIT License.
//
// Sts3215Bus / SerialPort のテスト。pty 上の偽サーボを使うので実機・実ポートは開かない。
#include <gtest/gtest.h>
#include <pty.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "so_arm101_hardware/serial_port.hpp"
#include "so_arm101_hardware/sts3215_bus.hpp"

using namespace so_arm101_hardware;  // NOLINT

namespace
{

enum class Fault {None, Silent, BadChecksum, ServoError, WrongId, JunkPrefix};

/// pty のマスタ側で STS3215 のように振る舞う。受け取った命令コードを全て記録する。
class FakeServos
{
public:
  FakeServos()
  {
    termios tio{};
    cfmakeraw(&tio);
    EXPECT_EQ(openpty(&master_, &slave_, nullptr, &tio, nullptr), 0);
    char name[128];
    EXPECT_EQ(ttyname_r(slave_, name, sizeof(name)), 0);
    path_ = name;
    thread_ = std::thread([this] {loop();});
  }
  ~FakeServos()
  {
    stop_ = true;
    thread_.join();
    ::close(master_);
    ::close(slave_);
  }
  const std::string & path() const {return path_;}
  void set_register(uint8_t id, uint8_t addr, uint16_t value, int len = 2)
  {
    std::lock_guard<std::mutex> g(m_);
    regs_[id][addr] = value & 0xFF;
    if (len == 2) {regs_[id][addr + 1] = value >> 8;}
  }
  void set_fault(uint8_t id, Fault f)
  {
    std::lock_guard<std::mutex> g(m_);
    fault_id_ = id;
    fault_ = f;
  }
  void clear_fault() {set_fault(0, Fault::None);}
  std::vector<uint8_t> instructions()
  {
    std::lock_guard<std::mutex> g(m_);
    return instr_;
  }

private:
  void loop()
  {
    std::vector<uint8_t> buf;
    while (!stop_) {
      pollfd p{master_, POLLIN, 0};
      if (::poll(&p, 1, 20) <= 0) {continue;}
      uint8_t tmp[256];
      const ssize_t n = ::read(master_, tmp, sizeof(tmp));
      if (n <= 0) {continue;}
      buf.insert(buf.end(), tmp, tmp + n);
      while (parse(&buf)) {}
    }
  }
  bool parse(std::vector<uint8_t> * b)
  {
    while (b->size() >= 2 && !((*b)[0] == 0xFF && (*b)[1] == 0xFF)) {b->erase(b->begin());}
    if (b->size() < 4) {return false;}
    const size_t total = (*b)[3] + 4u;
    if (b->size() < total) {return false;}
    std::vector<uint8_t> pkt(b->begin(), b->begin() + total);
    b->erase(b->begin(), b->begin() + total);
    handle(pkt);
    return true;
  }
  static uint8_t chk(const std::vector<uint8_t> & body)
  {
    unsigned s = 0;
    for (auto c : body) {s += c;}
    return static_cast<uint8_t>(~s & 0xFF);
  }
  void respond(uint8_t id, uint8_t addr, uint8_t len)
  {
    Fault f;
    uint8_t fid;
    std::map<uint8_t, uint8_t> mem;
    {
      std::lock_guard<std::mutex> g(m_);
      f = fault_;
      fid = fault_id_;
      mem = regs_[id];
    }
    const bool faulty = (fid == id);
    if (faulty && f == Fault::Silent) {return;}
    std::vector<uint8_t> body{static_cast<uint8_t>(faulty && f == Fault::WrongId ? id + 1 : id),
      static_cast<uint8_t>(len + 2), static_cast<uint8_t>(faulty && f == Fault::ServoError ? 0x20 : 0)};
    for (int i = 0; i < len; ++i) {body.push_back(mem.count(addr + i) ? mem[addr + i] : 0);}
    std::vector<uint8_t> out;
    if (faulty && f == Fault::JunkPrefix) {out = {0x12, 0xFF, 0x34};}
    out.push_back(0xFF);
    out.push_back(0xFF);
    out.insert(out.end(), body.begin(), body.end());
    uint8_t c = chk(body);
    if (faulty && f == Fault::BadChecksum) {c ^= 0x55;}
    out.push_back(c);
    ::write(master_, out.data(), out.size());
  }
  void handle(const std::vector<uint8_t> & pkt)
  {
    // FF FF id len inst params... chk
    const uint8_t id = pkt[2], inst = pkt[4];
    std::vector<uint8_t> body(pkt.begin() + 2, pkt.end() - 1);
    {
      std::lock_guard<std::mutex> g(m_);
      instr_.push_back(inst);
      last_chk_ok_ = (chk(body) == pkt.back());
    }
    if (inst == 0x02) {
      respond(id, pkt[5], pkt[6]);
    } else if (inst == 0x82) {
      for (size_t i = 7; i < pkt.size() - 1; ++i) {respond(pkt[i], pkt[5], pkt[6]);}
    }
  }

  int master_{-1}, slave_{-1};
  std::string path_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::mutex m_;
  std::map<uint8_t, std::map<uint8_t, uint8_t>> regs_;
  std::vector<uint8_t> instr_;
  uint8_t fault_id_{0};
  Fault fault_{Fault::None};
  bool last_chk_ok_{true};
};

class BusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (uint8_t id = 1; id <= 6; ++id) {
      servos.set_register(id, 3, 777);
      servos.set_register(id, 56, 2000 + id * 10);
    }
    std::string err;
    ASSERT_TRUE(port.open(servos.path(), 1000000, &err)) << err;
    bus = std::make_unique<Sts3215Bus>(port, std::chrono::milliseconds(30));
  }
  FakeServos servos;
  SerialPort port;
  std::unique_ptr<Sts3215Bus> bus;
  const std::vector<uint8_t> ids{1, 2, 3, 4, 5, 6};
};

}  // namespace

TEST(Packets, RequestBytesMatchScservoSdk)
{
  // READ id=1 addr=56 len=2: FF FF 01 04 02 38 02 CHK  (CHK = ~(01+04+02+38+02) = ~0x41 = 0xBE)
  const auto r = Sts3215Bus::build_request(1, Instruction::Read, {56, 2});
  EXPECT_EQ(r, (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
  // SYNC_READ ids 1,2: FF FF FE 06 82 38 02 01 02 CHK  (SDK: LEN = 台数+4 = 6、CHK = ~0xC3 = 0x3C)
  const auto s = Sts3215Bus::build_request(0xFE, Instruction::SyncRead, {56, 2, 1, 2});
  EXPECT_EQ(s, (std::vector<uint8_t>{0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x02, 0x01, 0x02, 0x3C}));
}

TEST(Packets, OnlyReadInstructionsExist)
{
  // 書き込み系の命令コード (WRITE=3, REG_WRITE=4, ACTION=5, SYNC_WRITE=0x83) は型として存在しない
  EXPECT_EQ(static_cast<int>(Instruction::Read), 0x02);
  EXPECT_EQ(static_cast<int>(Instruction::SyncRead), 0x82);
}

TEST(Serial, OpenFailuresAreReported)
{
  SerialPort p;
  std::string err;
  EXPECT_FALSE(p.open("/nonexistent/ttyXX", 1000000, &err));
  EXPECT_NE(err.find("open("), std::string::npos) << err;
  FakeServos f;
  EXPECT_FALSE(p.open(f.path(), 12345, &err));  // 非対応のボーレート
  EXPECT_NE(err.find("baudrate"), std::string::npos) << err;
}

TEST(Serial, SecondOpenIsRefusedByExclusiveLock)
{
  if (geteuid() == 0) {GTEST_SKIP() << "root は TIOCEXCL を無視する";}
  FakeServos f;
  SerialPort a, b;
  std::string err;
  ASSERT_TRUE(a.open(f.path(), 1000000, &err)) << err;
  EXPECT_FALSE(b.open(f.path(), 1000000, &err));  // 他のプロセスが開いていたら失敗する
}

TEST_F(BusTest, SyncReadReturnsAllPositions)
{
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 99;
  ASSERT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::Ok) << bus->last_error();
  EXPECT_EQ(failed, 0);
  ASSERT_EQ(out.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    ASSERT_EQ(out[i].size(), 2u);
    EXPECT_EQ(out[i][0] | (out[i][1] << 8), 2000 + int(ids[i]) * 10);
  }
}

TEST_F(BusTest, ReadU16AndSingleRead)
{
  uint16_t model = 0;
  ASSERT_EQ(bus->read_u16(3, reg::kModelNumber, &model), Status::Ok);
  EXPECT_EQ(model, kModelNumberSts3215);
  std::vector<uint8_t> d;
  ASSERT_EQ(bus->read(2, reg::kReturnDelayTime, &d), Status::Ok);
  EXPECT_EQ(d.size(), 1u);
}

TEST_F(BusTest, TimeoutReportsFailedId)
{
  servos.set_fault(3, Fault::Silent);
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::Timeout);
  EXPECT_EQ(failed, 3);
  EXPECT_FALSE(bus->last_error().empty());
}

TEST_F(BusTest, BadChecksumIsDetected)
{
  servos.set_fault(2, Fault::BadChecksum);
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::BadChecksum);
  EXPECT_EQ(failed, 2);
}

TEST_F(BusTest, CommErrorMessageCarriesRawBytesForDiagnosis)
{
  servos.set_fault(2, Fault::BadChecksum);
  std::vector<uint8_t> d;
  EXPECT_EQ(bus->read(2, reg::kPresentPosition, &d), Status::BadChecksum);
  const std::string & e = bus->last_error();
  EXPECT_NE(e.find("tx=FF FF 02 04 02 38 02"), std::string::npos) << e;  // 要求（Present_Position=56=0x38, 2 バイト）
  EXPECT_NE(e.find("rx=FF FF 02"), std::string::npos) << e;               // 受信したバイト
  EXPECT_NE(e.find("flushed_before_request="), std::string::npos) << e;
  EXPECT_TRUE(Sts3215Bus::is_comm_error(Status::BadChecksum));
  EXPECT_TRUE(Sts3215Bus::is_comm_error(Status::Timeout));
  EXPECT_TRUE(Sts3215Bus::is_comm_error(Status::BadFrame));
  EXPECT_FALSE(Sts3215Bus::is_comm_error(Status::ServoError));
  EXPECT_FALSE(Sts3215Bus::is_comm_error(Status::IdMismatch));
}

TEST_F(BusTest, ServoErrorByteIsDetected)
{
  servos.set_fault(5, Fault::ServoError);
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::ServoError);
  EXPECT_EQ(failed, 5);
}

TEST_F(BusTest, WrongIdIsDetected)
{
  servos.set_fault(6, Fault::WrongId);  // ID 6 が 7（リスト外）と名乗る
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::IdMismatch);
  EXPECT_EQ(failed, 6);
}

TEST_F(BusTest, LaterIdAnsweringFirstMeansMissingServo)
{
  servos.set_fault(4, Fault::WrongId);  // ID 4 が 5 と名乗る＝4 は応答していないのと同じ
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out, &failed), Status::Timeout);
  EXPECT_EQ(failed, 4);
}

TEST_F(BusTest, JunkBeforeHeaderIsSkipped)
{
  servos.set_fault(2, Fault::JunkPrefix);
  std::vector<std::vector<uint8_t>> out;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out), Status::Ok) << bus->last_error();
}

TEST_F(BusTest, RecoversAfterFailure)
{
  servos.set_fault(3, Fault::Silent);
  std::vector<std::vector<uint8_t>> out;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out), Status::Timeout);
  servos.clear_fault();
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out), Status::Ok) << bus->last_error();
}

TEST_F(BusTest, ClosedPortIsReported)
{
  port.close();
  std::vector<std::vector<uint8_t>> out;
  EXPECT_EQ(bus->sync_read(ids, reg::kPresentPosition, &out), Status::PortError);
}

TEST_F(BusTest, NeverSendsAnythingButReadInstructions)
{
  std::vector<std::vector<uint8_t>> out;
  uint16_t v;
  std::vector<uint8_t> d;
  bus->sync_read(ids, reg::kPresentPosition, &out);
  bus->read_u16(1, reg::kHomingOffset, &v);
  bus->read_u16(1, reg::kMinPositionLimit, &v);
  bus->read_u16(1, reg::kMaxPositionLimit, &v);
  bus->read(1, reg::kReturnDelayTime, &d);
  servos.set_fault(1, Fault::Silent);
  bus->read_u16(1, reg::kModelNumber, &v);  // 失敗した要求も含める
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto seen = servos.instructions();
  ASSERT_FALSE(seen.empty());
  for (uint8_t inst : seen) {
    EXPECT_TRUE(inst == 0x02 || inst == 0x82) << "unexpected instruction 0x" << std::hex << int(inst);
  }
}
