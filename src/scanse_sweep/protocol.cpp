#include <chrono>
#include <thread>

#include <scanse_sweep/protocol.hpp>

namespace sweep {
namespace protocol {

static uint8_t checksum_response_header(const response_header_s& v) {
  return ((v.cmdStatusByte1 + v.cmdStatusByte2) & 0x3F) + 0x30;
}

static uint8_t checksum_response_param(const response_param_s& v) {
  return ((v.cmdStatusByte1 + v.cmdStatusByte2) & 0x3F) + 0x30;
}

static uint8_t checksum_response_scan_packet(const response_scan_packet_s& v) {
  uint64_t checksum = 0;
  checksum += v.sync_error;
  checksum += v.angle & 0xff00;
  checksum += v.angle & 0x00ff;
  checksum += v.distance & 0xff00;
  checksum += v.distance & 0x00ff;
  checksum += v.signal_strength;
  return checksum % 255;
}

void write_command(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  cmd_packet_s packet;
  packet.cmdByte1 = cmd[0];
  packet.cmdByte2 = cmd[1];
  packet.cmdParamTerm = '\n';

  // pause for 2ms, so the device is never bombarded with back to back commands
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  serial::device_write(serial, &packet, sizeof(cmd_packet_s));
}

void write_command_with_arguments(serial::device_s serial, const uint8_t cmd[2], const uint8_t arg[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);
  SWEEP_ASSERT(arg);

  cmd_param_packet_s packet;
  packet.cmdByte1 = cmd[0];
  packet.cmdByte2 = cmd[1];
  packet.cmdParamByte1 = arg[0];
  packet.cmdParamByte2 = arg[1];
  packet.cmdParamTerm = '\n';

  serial::device_write(serial, &packet, sizeof(cmd_param_packet_s));
}

response_header_s read_response_header(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  response_header_s header;
  serial::device_read(serial, &header, sizeof(header));

  uint8_t checksum = checksum_response_header(header);

  if (checksum != header.cmdSum)
    throw error{"invalid response header checksum"};

  bool ok = header.cmdByte1 == cmd[0] && header.cmdByte2 == cmd[1];

  if (!ok)
    throw error{"invalid header response commands"};

  return header;
}

response_param_s read_response_param(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  response_param_s param;
  serial::device_read(serial, &param, sizeof(param));

  uint8_t checksum = checksum_response_param(param);

  if (checksum != param.cmdSum)
    throw error{"invalid response param header checksum"};

  bool ok = param.cmdByte1 == cmd[0] && param.cmdByte2 == cmd[1];

  if (!ok)
    throw error{"invalid param response commands"};

  return param;
}

response_scan_packet_s read_response_scan(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_scan_packet_s scan;
  serial::device_read(serial, &scan, sizeof(scan));

  uint8_t checksum = checksum_response_scan_packet(scan);

  if (checksum != scan.checksum)
    throw error{"invalid scan response commands"};

  return scan;
}

response_info_motor_ready_s read_response_info_motor_ready(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_motor_ready_s info;
  serial::device_read(serial, &info, sizeof(info));

  bool ok = info.cmdByte1 == MOTOR_READY[0] && info.cmdByte2 == MOTOR_READY[1];

  if (!ok)
    throw error{"invalid motor ready response commands"};

  return info;
}

response_info_motor_speed_s read_response_info_motor_speed(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_motor_speed_s info;
  serial::device_read(serial, &info, sizeof(info));

  bool ok = info.cmdByte1 == MOTOR_INFORMATION[0] && info.cmdByte2 == MOTOR_INFORMATION[1];

  if (!ok)
    throw error{"invalid motor info response commands"};

  return info;
}

response_info_sample_rate_s read_response_info_sample_rate(sweep::serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_sample_rate_s info;
  serial::device_read(serial, &info, sizeof(info));

  bool ok = info.cmdByte1 == SAMPLE_RATE_INFORMATION[0] && info.cmdByte2 == SAMPLE_RATE_INFORMATION[1];

  if (!ok)
    throw error{"invalid sample rate info response commands"};

  return info;
}

} // ns protocol
} // ns sweep
