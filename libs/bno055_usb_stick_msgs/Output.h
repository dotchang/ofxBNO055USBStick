#ifndef __BNO055_USB_STICK_MSGS_OUTPUT_H__
#define __BNO055_USB_STICK_MSGS_OUTPUT_H__

#define _WITHOUT_OPENFRAMEWORKS 0

#if _WITHOUT_OPENFRAMEWORKS
#define _USE_MATH_DEFINES
#include <math.h>

class ofVec3f {
public:
	double x, y, z;
};

class ofVec4f {
public:
	double x, y, z, w;
};

class ofQuaternion {
public:
	ofQuaternion() {}
	ofQuaternion(double _x, double _y, double _z, double _w) : x_(_x), y_(_y), z_(_z), w_(_w) {}
	double x() { return x_; }
	double y() { return y_; }
	double z() { return z_; }
	double w() { return w_; }
	double x_, y_, z_, w_;
};

#include <time.h>
typedef clock_t ofTime;

ofTime ofGetCurrentTime(){
	return clock();
}

#else
#include <ofMain.h>
#include <ofVec3f.h>
#include <ofVec4f.h>
#include <ofQuaternion.h>
#define M_PI PI
#endif

namespace bno055_usb_stick_msgs {
	class Output {
	public:
		Output() {};

		class Header {
		public:
			ofTime stamp;
			std::string frame_id;
		} header;
		ofVec3f acceleration;
		ofVec3f magnetometer;
		ofVec3f gyroscope;
		ofVec3f euler_angles;
		ofQuaternion quaternion;
		ofVec3f linear_acceleration;
		ofVec3f gravity_vector;
		double temperature;
		ofVec4f calibration_status;
	};
};

#endif // __BNO055_USB_STICK_MSGS_OUTPUT_H__