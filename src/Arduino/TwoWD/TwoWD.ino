#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "custom_msgs/TwoWDAngVel.h"
// #include "custom_msgs/WheelEnc.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

float cmd_ang_vel[2] = {0.0, 0.0};

std_msgs::Float32MultiArray gainL_msg; // PIDゲインを受け取るための配列
std_msgs::Float32MultiArray gainR_msg;
custom_msgs::TwoWDAngVel wheel_vel;
// エンコーダのCPR
int enc_cpr = 1028 * 4;

ros::NodeHandle nh;
ros::Publisher enc_pub("wheelVel", &wheel_vel);
// ros::Publisher gainL_pub("Arduino/gainL", &gainL_msg);
// ros::Publisher gainR_pub("Arduino/gainR", &gainR_msg);

// 単位は[count/sec]
void angVelCb(const custom_msgs::TwoWDAngVel &ang_vel_msg)
{
	// ROSから来るのが100オーダーの値で、速度PIDの目標速度のオーダーが10くらいなので10で割ってる
	// ロボットにとって前方向への移動のとき左タイヤは正方向、右タイヤは負方向に回転するので符号をつけて補正している。
	cmd_ang_vel[LEFT_MOTOR] = ang_vel_msg.L / 10;
	cmd_ang_vel[RIGHT_MOTOR] = -ang_vel_msg.R / 10;
}

ros::Subscriber<custom_msgs::TwoWDAngVel> ang_vel_sub("arduino/cmd_w", &angVelCb);

void setup()
{
	Cubic::begin();
	// Serial.begin(115200);

	nh.getHardware()->setBaud(80000000);
	nh.initNode();

	gainL_msg.data_length = 3;
	gainL_msg.data = (float *)malloc(sizeof(float) * 3);
	gainR_msg.data_length = 3;
	gainR_msg.data = (float *)malloc(sizeof(float) * 3);

	// ROSとつながるまで待つ
	while (!nh.connected())
	{
		nh.spinOnce();
		delay(10);
	}

	if (!nh.getParam("/arduino/gainL", gainL_msg.data, 3))
	{
		// default values
		gainL_msg.data[0] = 7.0; // P
		gainL_msg.data[1] = 1.0; // I
		gainL_msg.data[2] = 0.3; // D
	}
	if (!nh.getParam("/arduino/gainR", gainR_msg.data, 3))
	{
		// default values
		gainR_msg.data[0] = 7.0; // P
		gainR_msg.data[1] = 1.0; // I
		gainR_msg.data[2] = 0.3; // D
	}
	nh.getParam("/twoWD/enc_cpr", &enc_cpr);

	nh.subscribe(ang_vel_sub);
	nh.advertise(enc_pub);
	// nh.advertise(gainL_pub);
	// nh.advertise(gainR_pub);
}

void loop()
{
	nh.spinOnce();
	static bool stopFlag = false;

	//  nh.getParam("/Arduino/gainL", gainL_msg.data, 3);
	//  nh.getParam("/Arduino/gainR", gainR_msg.data, 3);

	static Cubic_controller::Velocity_PID velocityPID[] = {
		{0, 0, Cubic_controller::encoderType::inc, enc_cpr, 0.7, 0.5, gainL_msg.data[0], gainL_msg.data[1], gainL_msg.data[2], 0.0, false, false}, // L
		{1, 1, Cubic_controller::encoderType::inc, enc_cpr, 0.7, 0.5, gainR_msg.data[0], gainR_msg.data[1], gainR_msg.data[2], 0.0, false, false}, // R
	};

	if (cmd_ang_vel[LEFT_MOTOR] == 0 && cmd_ang_vel[RIGHT_MOTOR] == 0)
	{
		stopFlag = true;
	}
	else
	{
		stopFlag = false;
		velocityPID[LEFT_MOTOR].setTarget(cmd_ang_vel[LEFT_MOTOR]);
		velocityPID[RIGHT_MOTOR].setTarget(cmd_ang_vel[RIGHT_MOTOR]);
	}
  
  static bool stop_log_trigger = true; // 状態が遷移したときのみログを出すための制御用変数

	if (stopFlag)
	{
    if(stop_log_trigger){
		// Serial.println("stopping...");
		    nh.loginfo("Stopping...");
        stop_log_trigger = false;
    }

		velocityPID[LEFT_MOTOR].reset();
		velocityPID[RIGHT_MOTOR].reset();

		for (int i = 0; i < 8; i++)
		{
			DC_motor::put(i, 0);
		}
		wheel_vel.L = 0;
		wheel_vel.R = 0;
	}
	else
	{
    if(!stop_log_trigger){
		    nh.loginfo("Culculating");
        stop_log_trigger = true;
    }
		wheel_vel.L = velocityPID[LEFT_MOTOR].compute();
		wheel_vel.R = -velocityPID[RIGHT_MOTOR].compute();
	}

	// wheel_vel.L = velocityPID[LEFT_MOTOR].encoderToAngle(velocityPID[LEFT_MOTOR].readEncoder()) * radius;
	// wheel_vel.R = velocityPID[RIGHT_MOTOR].readEncoder();

	enc_pub.publish(&wheel_vel);
	//  gainL_pub.publish(&gainL_msg);
	//  gainR_pub.publish(&gainR_msg);
	Cubic::update();
}
