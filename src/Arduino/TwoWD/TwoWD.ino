#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include "custom_msgs/TwoWDAngVel.h"
#include "custom_msgs/WheelEnc.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

float ang_vel[2] = {0,0}; 

custom_msgs::WheelEnc wheel_enc;

ros::NodeHandle nh;
ros::Publisher enc_pub("TwoWDEncoder", &wheel_enc);

// 単位は[count/sec]
void angVelCb(const custom_msgs::TwoWDAngVel& ang_vel_msg){
	ang_vel[0] = ang_vel_msg.L;
	ang_vel[1] = ang_vel_msg.R;
}

ros::Subscriber<custom_msgs::TwoWDAngVel> ang_vel_sub("TwoWDAngVel", &angVelCb);

void setup()
{
	Cubic::begin();
	// Serial.begin(115200);

	nh.getHardware()->setBaud(80000000);
	nh.initNode();

	nh.subscribe(ang_vel_sub);
	nh.advertise(enc_pub);
}

void loop()
{
	nh.spinOnce();
	static bool stopFlag = false;

	static Cubic_controller::Velocity_PID velocityPID[]= {
		{4, 2, Cubic_controller::encoderType::inc, 4096,0.7, 0.5, 0.08, 0.0, 0.0001, 0.1, false, true}, // L
		{2, 1, Cubic_controller::encoderType::inc, 4096,0.7, 0.5, 0.08, 0.0, 0.0001, 0.1, false, true}, // R
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