// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/sts3215_bus.hpp"

#include <algorithm>
#include <cstdio>

namespace so_arm101_hardware
{

namespace
{
constexpr uint8_t kBroadcastId = 0xFE;  // scservo_def.py BROADCAST_ID
constexpr uint8_t kMaxId = 0xFC;        // scservo_def.py MAX_ID
constexpr int kMaxJunkBytes = 64;       // ヘッダ探索で読み捨てる上限
}  // namespace

const char * to_string(Status s)
{
  switch (s) {
    case Status::Ok: return "ok";
    case Status::Timeout: return "timeout";
    case Status::BadChecksum: return "bad checksum";
    case Status::BadFrame: return "bad frame";
    case Status::IdMismatch: return "id mismatch";
    case Status::ServoError: return "servo error";
    case Status::PortError: return "port error";
  }
  return "?";
}

int decode_sign_magnitude(uint16_t encoded, int sign_bit)
{
  const int direction_bit = (encoded >> sign_bit) & 1;
  const int magnitude = encoded & ((1 << sign_bit) - 1);
  return direction_bit ? -magnitude : magnitude;
}

bool encode_sign_magnitude(int v, int sign_bit, uint16_t * encoded)
{
  const int mag = v < 0 ? -v : v;
  if (mag >= (1 << sign_bit)) {return false;}
  *encoded = static_cast<uint16_t>(mag | (v < 0 ? (1 << sign_bit) : 0));
  return true;
}

std::string describe_servo_error(uint8_t err)
{
  static const char * names[] = {"voltage", "sensor", "temperature", "current", "angle", "overload"};
  std::string s;
  for (int b = 0; b < 6; ++b) {
    if (err & (1 << b)) {s += std::string(s.empty() ? "" : "+") + names[b];}
  }
  if (err & 0xC0) {s += std::string(s.empty() ? "" : "+") + "unknown-bits";}
  return s.empty() ? "none" : s;
}

uint8_t Sts3215Bus::checksum(const uint8_t * body, size_t len)
{
  unsigned sum = 0;
  for (size_t i = 0; i < len; ++i) {sum += body[i];}
  return static_cast<uint8_t>(~sum & 0xFF);
}

std::vector<uint8_t> Sts3215Bus::build_request(
  uint8_t id, Instruction inst, const std::vector<uint8_t> & params)
{
  return build_packet(id, static_cast<uint8_t>(inst), params);
}

std::vector<uint8_t> Sts3215Bus::build_packet(
  uint8_t id, uint8_t inst, const std::vector<uint8_t> & params)
{
  // FF FF <ID> <LEN> <INST> <PARAM…> <CHK>、LEN = PARAM 数 + 2
  std::vector<uint8_t> pkt;
  pkt.reserve(params.size() + 6);
  pkt.push_back(0xFF);
  pkt.push_back(0xFF);
  pkt.push_back(id);
  pkt.push_back(static_cast<uint8_t>(params.size() + 2));
  pkt.push_back(inst);
  pkt.insert(pkt.end(), params.begin(), params.end());
  pkt.push_back(checksum(pkt.data() + 2, pkt.size() - 2));
  return pkt;
}

static std::string hex_bytes(const std::vector<uint8_t> & v)
{
  std::string out;
  char buf[4];
  for (size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%02X", v[i]);
    if (i) {out += ' ';}
    out += buf;
  }
  return out;
}

void Sts3215Bus::begin_request(const std::vector<uint8_t> & req)
{
  flushed_before_request_ = port_.flush_input();
  last_request_ = req;
  rx_bytes_.clear();
}

Status Sts3215Bus::fail(Status s, const std::string & msg)
{
  last_error_ = msg;
  if (is_comm_error(s)) {
    // 原因の切り分け用: 送った要求・受け取ったバイト・要求の前に捨てた古いバイト数
    last_error_ += " [tx=" + hex_bytes(last_request_) + " rx=" + hex_bytes(rx_bytes_) +
      " flushed_before_request=" + std::to_string(flushed_before_request_) + "]";
  }
  last_servo_error_ = 0;
  if (!defer_settle_) {
    // 次の要求で再同期できるように。通信エラー・ID 違いのときは、これから届く古い応答（SYNC_READ の残りなど）を
    // 待って捨てる（捨てた「後」に届いて、次の読み取りに入るのを防ぐ）。それ以外は捨てるだけ。
    // 何も受け取らずにタイムアウトしたとき（バスは黙っている）は、待っても何も来ないので捨てるだけ。
    const bool silent_bus = (s == Status::Timeout && rx_bytes_.empty());
    settle((is_comm_error(s) || s == Status::IdMismatch) && !silent_bus ? drain_quiet_ : std::chrono::microseconds(0));
  }
  return s;
}

void Sts3215Bus::settle(std::chrono::microseconds quiet)
{
  if (quiet.count() > 0) {
    port_.drain_until_quiet(quiet, drain_max_);
  } else {
    port_.flush_input();
  }
}

bool Sts3215Bus::discard_frame(std::chrono::microseconds timeout)
{
  uint8_t b = 0;
  int ff = 0, junk = 0;
  while (ff < 2) {
    if (port_.read_exact(&b, 1, timeout) != 1) {return false;}
    if (b == 0xFF) {++ff;} else {ff = 0; if (++junk > kMaxJunkBytes) {return false;}}
  }
  uint8_t head[2];
  if (port_.read_exact(head, 2, timeout) != 2) {return false;}
  if (head[1] < 2 || head[1] > 250) {return false;}
  uint8_t rest[256];
  return port_.read_exact(rest, head[1], timeout) == head[1];
}

Status Sts3215Bus::receive_status(uint8_t expect_id, size_t data_len, std::vector<uint8_t> * data)
{
  // ヘッダ FF FF を探す
  uint8_t b = 0;
  int ff_count = 0;
  int junk = 0;
  while (ff_count < 2) {
    if (port_.read_exact(&b, 1, timeout_) != 1) {
      return fail(Status::Timeout, "timeout waiting for header (id " + std::to_string(expect_id) + ")");
    }
    rx_bytes_.push_back(b);
    if (b == 0xFF) {
      ++ff_count;
    } else {
      ff_count = 0;
      if (++junk > kMaxJunkBytes) {return fail(Status::BadFrame, "no header in received bytes");}
    }
  }
  uint8_t head[2];  // ID, LEN
  const size_t got_head = port_.read_exact(head, 2, timeout_);
  rx_bytes_.insert(rx_bytes_.end(), head, head + got_head);
  if (got_head != 2) {
    return fail(Status::Timeout, "timeout reading id/len (id " + std::to_string(expect_id) + ")");
  }
  const uint8_t id = head[0];
  const uint8_t len = head[1];
  if (len < 2 || len > 250) {
    return fail(Status::BadFrame, "invalid length " + std::to_string(len));
  }
  std::vector<uint8_t> rest(len);  // ERR + DATA + CHK
  const size_t got_rest = port_.read_exact(rest.data(), len, timeout_);
  rx_bytes_.insert(rx_bytes_.end(), rest.begin(), rest.begin() + got_rest);
  if (got_rest != len) {
    return fail(Status::Timeout, "timeout reading payload (id " + std::to_string(expect_id) + ")");
  }
  std::vector<uint8_t> body;
  body.reserve(len + 2u);
  body.push_back(id);
  body.push_back(len);
  body.insert(body.end(), rest.begin(), rest.end() - 1);
  if (checksum(body.data(), body.size()) != rest.back()) {
    return fail(Status::BadChecksum, "checksum mismatch (id " + std::to_string(id) + ")");
  }
  last_rx_id_ = id;
  if (id != expect_id) {
    return fail(
      Status::IdMismatch,
      "expected id " + std::to_string(expect_id) + ", got " + std::to_string(id));
  }
  const uint8_t err = rest[0];
  if (err != 0) {
    last_error_.clear();
    char hex[8];
    std::snprintf(hex, sizeof(hex), "0x%02X", err);
    const Status st = fail(
      Status::ServoError, "servo " + std::to_string(id) + " reported error " + hex + " (" +
      describe_servo_error(err) + ")");
    last_servo_error_ = err;
    return st;
  }
  if (static_cast<size_t>(len) != data_len + 2) {
    return fail(
      Status::BadFrame, "unexpected data length " + std::to_string(len - 2) + " (expected " +
      std::to_string(data_len) + ")");
  }
  data->assign(rest.begin() + 1, rest.end() - 1);
  return Status::Ok;
}

Status Sts3215Bus::read(uint8_t id, Register r, std::vector<uint8_t> * data)
{
  SerialPort::Transaction tx(port_);
  if (!port_.is_open()) {return fail(Status::PortError, "port not open");}
  if (id > kMaxId) {return fail(Status::BadFrame, "invalid id");}
  const auto req = build_request(id, Instruction::Read, {r.address, r.length});
  begin_request(req);
  if (!port_.write_all(req.data(), req.size())) {return fail(Status::PortError, "write failed");}
  return receive_status(id, r.length, data);
}

Status Sts3215Bus::read_u16(uint8_t id, Register r, uint16_t * value)
{
  std::vector<uint8_t> d;
  const Status s = read(id, r, &d);
  if (s != Status::Ok) {return s;}
  if (d.size() != 2) {return fail(Status::BadFrame, "expected 2 bytes");}
  *value = static_cast<uint16_t>(d[0] | (d[1] << 8));  // リトルエンディアン
  return Status::Ok;
}

Status Sts3215Bus::sync_read(
  const std::vector<uint8_t> & ids, Register r, std::vector<std::vector<uint8_t>> * out,
  uint8_t * failed_id)
{
  if (failed_id) {*failed_id = 0;}
  SerialPort::Transaction tx(port_);
  if (!port_.is_open()) {return fail(Status::PortError, "port not open");}
  if (ids.empty() || ids.size() > 32) {return fail(Status::BadFrame, "invalid id list");}
  std::vector<uint8_t> params{r.address, r.length};
  params.insert(params.end(), ids.begin(), ids.end());
  const auto req = build_request(kBroadcastId, Instruction::SyncRead, params);
  begin_request(req);
  if (!port_.write_all(req.data(), req.size())) {return fail(Status::PortError, "write failed");}
  out->assign(ids.size(), {});
  struct Defer  // 途中の失敗では後始末をせず、最後に 1 回だけ行う
  {
    bool & f;
    explicit Defer(bool & x) : f(x) {f = true;}
    ~Defer() {f = false;}
  } defer(defer_settle_);
  for (size_t i = 0; i < ids.size(); ++i) {
    last_rx_id_ = -1;
    Status s = receive_status(ids[i], r.length, &(*out)[i]);
    size_t consumed_upto = i;  // ids[0..consumed_upto] の応答は、受け取った（または、待ったが来ない）
    if (s == Status::IdMismatch) {
      const auto later = std::find(ids.begin() + i + 1, ids.end(), static_cast<uint8_t>(last_rx_id_));
      if (later != ids.end()) {
        // 後ろの ID が先に返った＝この ID は応答していない
        last_error_ = "no answer from id " + std::to_string(ids[i]) + " (id " +
          std::to_string(last_rx_id_) + " answered first)";
        s = Status::Timeout;
        consumed_upto = static_cast<size_t>(later - ids.begin());
      }
    }
    if (s != Status::Ok) {
      if (failed_id) {*failed_id = ids[i];}
      // 案 B: 残りの ID の応答がこれから届く。読み切ってから（案 A）静かになるまで待って捨てる
      auto quiet = drain_quiet_;
      if (s == Status::Timeout && rx_bytes_.empty() && consumed_upto == i) {
        // タイムアウトの間（既定 10 ms）に 1 バイトも来なかった＝バスは黙っている。残りの応答も、これから届く
        // 古い応答も無い（あれば、10 ms の間に届いている）ので、待たずに捨てるだけ（待つと周期が延びる）。
        quiet = std::chrono::microseconds(0);
      } else if (drain_quiet_.count() > 0 && (is_comm_error(s) || s == Status::IdMismatch || s == Status::ServoError)) {
        bool all = true;
        const auto budget_end = std::chrono::steady_clock::now() + drain_max_;
        for (size_t k = consumed_upto + 1; k < ids.size(); ++k) {
          if (std::chrono::steady_clock::now() >= budget_end || !discard_frame(drain_quiet_)) {all = false; break;}
        }
        if (all) {quiet = std::min(quiet, std::chrono::microseconds(500));}
      } else if (!(is_comm_error(s) || s == Status::IdMismatch)) {
        quiet = std::chrono::microseconds(0);
      }
      defer_settle_ = false;
      settle(quiet);
      return s;
    }
  }
  return Status::Ok;
}

}  // namespace so_arm101_hardware
