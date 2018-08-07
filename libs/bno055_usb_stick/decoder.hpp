#ifndef BNO055_USB_STICK_DECODER_HPP
#define BNO055_USB_STICK_DECODER_HPP

#include <algorithm>
#include <string>

#include "bno055_usb_stick/constants.hpp"
#include "bno055_usb_stick_msgs/Output.h"

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {
class Decoder {
public:
  Decoder(const std::string &ns)
      : frame_id_("bno055") {}

  virtual ~Decoder() {}

  bno055_usb_stick_msgs::Output decode(const boost::uint8_t *data) {
    bno055_usb_stick_msgs::Output output;
    output.header.stamp = ofGetCurrentTime(); // ros::Time::now();
    output.header.frame_id = frame_id_;
    output.acceleration = decodeAcc(data + Constants::ACC_POS);
    output.magnetometer = decodeMag(data + Constants::MAG_POS);
    output.gyroscope = decodeGyr(data + Constants::GYR_POS);
    output.euler_angles = decodeEul(data + Constants::EUL_POS);
    output.quaternion = decodeQua(data + Constants::QUA_POS);
    output.linear_acceleration = decodeLia(data + Constants::LIA_POS);
    output.gravity_vector = decodeGrv(data + Constants::GRV_POS);
    output.temperature = decodeTemp(data + Constants::TEMP_POS);
    output.calibration_status = decodeCalibStat(data + Constants::CALIB_STAT_POS);
    return output;
  }

private:
  static ofVec3f decodeAcc(const boost::uint8_t *data) {
    ofVec3f acc;
    acc.x = decodeVal(data[1], data[0], Constants::ACC_DENOM);
    acc.y = decodeVal(data[3], data[2], Constants::ACC_DENOM);
    acc.z = decodeVal(data[5], data[4], Constants::ACC_DENOM);
    return acc;
  }

  static ofVec3f decodeMag(const boost::uint8_t *data) {
    ofVec3f mag;
    mag.x = decodeVal(data[1], data[0], Constants::MAG_DENOM);
    mag.y = decodeVal(data[3], data[2], Constants::MAG_DENOM);
    mag.z = decodeVal(data[5], data[4], Constants::MAG_DENOM);
    return mag;
  }

  static ofVec3f decodeGyr(const boost::uint8_t *data) {
    ofVec3f gyr;
    gyr.x = decodeVal(data[1], data[0], Constants::GYR_DENOM) * M_PI / 180.;
    gyr.y = decodeVal(data[3], data[2], Constants::GYR_DENOM) * M_PI / 180.;
    gyr.z = decodeVal(data[5], data[4], Constants::GYR_DENOM) * M_PI / 180.;
    return gyr;
  }

  static ofVec3f decodeEul(const boost::uint8_t *data) {
    ofVec3f eul;
    eul.x = decodeVal(data[1], data[0], Constants::EUL_DENOM) * M_PI / 180.; // heading
    eul.y = decodeVal(data[3], data[2], Constants::EUL_DENOM) * M_PI / 180.; // roll
    eul.z = decodeVal(data[5], data[4], Constants::EUL_DENOM) * M_PI / 180.; // pitch
    return eul;
  }

  static ofQuaternion decodeQua(const boost::uint8_t *data) {
    float w, x, y, z;
    w = decodeVal(data[1], data[0], Constants::QUA_DENOM);
    x = decodeVal(data[3], data[2], Constants::QUA_DENOM);
    y = decodeVal(data[5], data[4], Constants::QUA_DENOM);
    z = decodeVal(data[7], data[6], Constants::QUA_DENOM);
    return ofQuaternion(x, y, z, w);
  }

  static ofVec3f decodeLia(const boost::uint8_t *data) {
    ofVec3f lia;
    lia.x = decodeVal(data[1], data[0], Constants::LIA_DENOM);
    lia.y = decodeVal(data[3], data[2], Constants::LIA_DENOM);
    lia.z = decodeVal(data[5], data[4], Constants::LIA_DENOM);
    return lia;
  }

  static ofVec3f decodeGrv(const boost::uint8_t *data) {
    ofVec3f grv;
    grv.x = decodeVal(data[1], data[0], Constants::GRV_DENOM);
    grv.y = decodeVal(data[3], data[2], Constants::GRV_DENOM);
    grv.z = decodeVal(data[5], data[4], Constants::GRV_DENOM);
    return grv;
  }

  static double decodeTemp(const boost::uint8_t *data) { return data[0] / Constants::TEMP_DENOM; }

  static ofVec4f decodeCalibStat(const boost::uint8_t *data) {
    ofVec4f calib_stat;
    calib_stat.x = (*data >> 6) & 0x3; // system
    calib_stat.y = (*data >> 4) & 0x3; // gyroscope
    calib_stat.z = (*data >> 2) & 0x3; // accelerometer
    calib_stat.w = *data & 0x3; // magnetometer
    return calib_stat;
  }

  static double decodeVal(const boost::uint8_t msb, const boost::uint8_t lsb, const double denom) {
    return boost::int16_t((boost::int16_t(msb) << 8) | lsb) / denom;
  }

private:
  const std::string frame_id_;
};
}
#endif // BNO055_USB_STICK_DECODER_HPP