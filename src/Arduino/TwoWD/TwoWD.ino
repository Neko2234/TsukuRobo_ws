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
  // ROSから来るのが100オーダーの値で、速度PIDの目標速度のオーダーが10くらいなので10で割ってる
  // ロボットにとって前方向への移動のとき左タイヤは正方向、右タイヤは負方向に回転するので符号をつけて補正している。
	ang_vel[0] = static_cast<float>(ang_vel_msg.L) / 10;
	ang_vel[1] = -static_cast<float>(ang_vel_msg.R) / 10;
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
		{0, 0, Cubic_controller::encoderType::inc, 2048 * 4, 0.8, 0.5, 1.0, 0.1, 0.1, 0.0, false, false}, // L
		{1, 1, Cubic_controller::encoderType::inc, 2048 * 4, 0.8, 0.5, 1.0, 0.1, 0.1, 0.0, false, false}, // R
	};

	velocityPID[LEFT_MOTOR].setTarget(ang_vel[0]);
	velocityPID[RIGHT_MOTOR].setTarget(ang_vel[1]);

	wheel_enc.L = Inc_enc::get(LEFT_MOTOR);
	wheel_enc.R = Inc_enc::get(RIGHT_MOTOR);

	enc_pub.publish(&wheel_enc);

	if (stopFlag)
	{
		// Serial.println("stopping...");

		velocityPID[LEFT_MOTOR].reset();
		velocityPID[RIGHT_MOTOR].reset();
		
		for (int i = 0; i < 8; i++)
		{
			DC_motor::put(i, 0);
		}
	}
	else
	{
		velocityPID[LEFT_MOTOR].compute();
		velocityPID[RIGHT_MOTOR].compute();
	}
	Cubic::update();
}