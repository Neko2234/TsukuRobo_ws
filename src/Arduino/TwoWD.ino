/* This is a sample program for Cubic Control Library. */
#include <Arduino.h>
#include <ros.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"
#include "custom_msgs/TwoWDAngVel.h"
#include "custom_msgs/WheelEnc.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

float ang_vel[2] = {0,0}; 

custom_msgs::WheelEnc wheel_enc;

ros::NodeHandle nh;
ros::Publisher enc_pub("TwoWDEncoder", &enc_msg);
ros::Subscriber<custom_msgs::TwoWDAngVel> ang_vel_sub("TwoWDAngVel", &angVelCb);

// 単位は[count/sec]
void angVelCb(const custom_msgs::TwoWDAngVel& ang_vel_msg){
	ang_vel[0] = ang_vel_msg.L;
	ang_vel[1] = ang_vel_msg.R;
}

void setup()
{
	Cubic::begin();
	// Serial.begin(115200);

	nh.getHardware()->setBaud(80000000);
	nh.initNode();

	nh.subscribe(enc_sub);
	nh.advertise(ang_vel_pub);
}

void loop()
{
	nh.spinOnce();
	static bool stopFlag = false;

	static Cubic_controller::Velocity_PID velocityPID[]= {
		{4, 2, encoderType::inc, 2048,0.7, 0.5, 0.08, 0.0, 0.0001, 0.1, false, true}, // L
		{2, 1, encoderType::inc, 2048,0.7, 0.5, 0.08, 0.0, 0.0001, 0.1, false, true}, // R
	};

	velocityPID[0].setTarget(ang_vel[0]);
	velocityPID[1].setTarget(ang_vel[1]);

	wheel_enc.L = Inc_enc::get(LEFT_MOTOR);
	wheel_enc.R = Inc_enc::get(RIGHT_MOTOR);

	enc_pub.publish(&wheel_enc);

	if (stopFlag)
	{
		// Serial.println("stopping...");

		velocityPID[0].reset();
		velocityPID[1].reset();
		
		for (int i = 0; i < 8; i++)
		{
			DC_motor::put(i, 0);
		}
	}
	else
	{
		velocityPID[0].compute();
		velocityPID[1].compute();
	}
	Cubic::update();
}