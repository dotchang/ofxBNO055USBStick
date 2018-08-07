#pragma once

#include "bno055_usb_stick/bno055_usb_stick.hpp"
#include "bno055_usb_stick/decoder.hpp"
#include "bno055_usb_stick_msgs/Output.h"

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "ofMain.h"
#include "ofThread.h"
#include "ofEasyCam.h"
#include "ofxGui.h" 

class ofxBNO055USBStickThread : public ofThread
{
public:
	void setup() {
		const std::string fixed_frame_id("fixed");
		device = std::make_shared<bno055_usb_stick::BNO055USBStick>(asio_service, boost::bind(&ofxBNO055USBStickThread::publish, this, _1, fixed_frame_id));
	}

	void threadedFunction() {
		while (1) {
			asio_service.run_one();
		}
	}

	void publish(const bno055_usb_stick_msgs::Output &output, const std::string &fixed_frame_id) {
		mutex.lock();
		current = output;
		mutex.unlock();
	}

	bno055_usb_stick_msgs::Output & getOutput() { return current;  }

protected:
	boost::asio::io_service asio_service;
	std::shared_ptr<bno055_usb_stick::BNO055USBStick> device;

	ofMutex mutex;
	bno055_usb_stick_msgs::Output current;
};

class ofxBNO055USBStick {
	public:
		ofxBNO055USBStick(){}

		void setup();
		void update();
		void draw();

		ofxBNO055USBStickThread bno055;
		bno055_usb_stick_msgs::Output output;

		ofEasyCam cam;
		ofLight light;
		ofBoxPrimitive box;
		ofCylinderPrimitive cylinder;
	
		ofxPanel gui;
		ofParameter<ofVec3f> acceleration;
		ofParameter<ofVec3f> magnetometer;
		ofParameter<ofVec3f> gyroscope;
		ofParameter<ofVec3f> euler_angles;
		ofParameter<ofQuaternion> quaternion;
		ofParameter<ofVec3f> linear_acceleration;
		ofParameter<ofVec3f> gravity_vector;
		ofParameter<double> temperature;
		ofParameter<ofVec4f> calibration_status;
};
