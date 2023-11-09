#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include "custom_msgs/TwoWDAngVel.h"
#include "custom_msgs/WheelEnc.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

float ang_vel[2] = {0.0, 0.0};

custom_msgs::WheelEnc wheel_enc;

ros::NodeHandle nh;
ros::Publisher enc_pub("TwoWD/encoder", &wheel_enc);

// 単位は[count/sec]
void angVelCb(const custom_msgs::TwoWDAngVel& ang_vel_msg){
	ang_vel[LEFT_MOTOR] = ang_vel_msg.L / 10;
	ang_vel[RIGHT_MOTOR] = -ang_vel_msg.R / 10;
}

ros::Subscriber<custom_msgs::TwoWDAngVel> ang_vel_sub("arduino/cmd_w", &angVelCb);

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
		{0, 0, Cubic_controller::encoderType::inc, 2048*4,0.7, 0.5, 1.0, 0.1, 0.1, 0.0, false, true}, // L
		{1, 1, Cubic_controller::encoderType::inc, 2048*4,0.7, 0.5, 1.0, 0.1, 0.1, 0.0, false, true}, // R
	};

	velocityPID[0].setTarget(ang_vel[LEFT_MOTOR]);
	velocityPID[1].setTarget(ang_vel[RIGHT_MOTOR]);

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
