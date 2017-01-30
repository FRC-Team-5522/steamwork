#include <unistd.h>
#include "WPILib.h"
#include <stdio.h>
#include <sys/time.h>
#include <AnalogGyro.h>
#include <iostream>
#include <math.h>
#include <ADXRS450_Gyro.h>
class Robot : public SampleRobot
{
	public:std::shared_ptr<NetworkTable> table;
	Joystick m_stick1;
	Joystick m_stick2;
	AnalogInput ultrasonic;
	DigitalInput di;
	ADXRS450_Gyro gyro;
	int bigshit;
	int smallshit;
	float compensate;
	Talon MLeftFront;
	Talon MLeftRear;
	Talon MRightFront;
	Talon MRightRear;
	const double kUpdatePeriod = 0.005;
	const double valueToCMs = 0.3175;
/*	void autoTargeting()
	{
		std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());
		std::vector<double> centerXs = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		std::vector<double> centerYs = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
		int max_i = -1;
		int i;
		double max_area = -1;
		if(areas.size() < 1)
		{
			printf("areas=%d\n", areas.size());
		}
		else
		{
			for (i = 0; i < areas.size(); i++)
			{
				if (areas[i] > max_area) {
					max_area = areas[i];
					max_i = i;
				}
			}
			float s = fabs(centerYs[max_i] - 240) * 0.003;
			printf("max_i=%d", max_i);
			if(centerXs[max_i] > 322)
			{
				MLeftFront.Set(-0.2);
				MLeftRear.Set(-0.2);
				MRightFront.Set(-0.2);
				MRightRear.Set(-0.2);
				//printf("centerXs=%f\n", centerXs[max_i]);
				//printf("TurnRight\n");
			} else if(centerXs[max_i] < 318)
			{
				MLeftFront.Set(0.2);
				MLeftRear.Set(0.2);
				MRightFront.Set(0.2);
				MRightRear.Set(0.2);
				//printf("centerXs=%f\n", centerXs[max_i]);
				//printf("TurnLeft\n");
			} else
			{
				MLeftFront.Set(0);
				MLeftRear.Set(0);
				MRightFront.Set(0);
				MRightRear.Set(0);
				bigshit = 1;
			}
			if(centerYs[max_i] > 241)
			{
				AngleModulator.Set(-0.2-s);
				printf("MakeitDown s=%f\n", s);
			}else if(centerYs[max_i] < 239 + conpensating())
			{
				AngleModulator.Set(0.2+s);
				printf("MakeitUp s=%f\n", s);
			}else
			{
				AngleModulator.Set(0);
				smallshit = 1;
			}
		}
	}*/
	void NOSmode()
	{
		MLeftFront.Set(m_stick1.GetY()*-0.285);
		MLeftRear.Set(m_stick1.GetY()*-0.285);
		MRightFront.Set(m_stick1.GetThrottle()*0.285);
		MRightRear.Set(m_stick1.GetThrottle()*0.285);
	}
	void allStop()
	{
		MLeftFront.Set(0);
		MLeftRear.Set(0);
		MRightFront.Set(0);
		MRightRear.Set(0);
	}
public:
	Robot() :
	    m_stick1(0),
		m_stick2(1),
		MLeftFront(1),
		MLeftRear(4),
		MRightFront(2),
		MRightRear(3),
		gyro(),
		di(1),
		ultrasonic(3)
	{
	}
	void RobotInit()
		{
			if (fork() == 0)
			{
				system("/bin/sh /home/lvuser/start-grip.sh");
				return;
				// #/bin/sh
				// ps | grep grip | grep -v grep | awk '{print $1}' | xargs kill
				// /usr/local/frc/JRE//bin/java -Xmx50m -XX:-OmitStackTraceInFastThrow -XX:+HeapDumpOnOutOfMemoryError -jar '/home/lvuser/grip.jar' '/home/lvuser/project.grip' > /dev/null
			}
			printf("shit\n");
			table = NetworkTable::GetTable("/GRIP/Shit");
			compensate = 0;
			gyro.Calibrate();
			printf("ManualMode\n");
		}
	void Test()
	{
		gyro.Calibrate();//calibration needs 3-4sec, so put this line into RobotInit.
		//Warning!!!算了用中文写 注意！！！因为calibration的时候机器人绝对绝对！！！绝对！！！不能动！！！所以要确保机器人稳稳的不动的时候再开电
		//开店之后Calibration之后 比赛那边会调试很久很久 累积误差会增大 所以在auto开始的时候先reset一下！！！谨记！！！！！！！！
		while(IsTest() && IsEnabled())
		{
			/*printf("%f cm\n", (ultrasonic.GetVoltage() * 102.3));
			Wait(0.1);*/
			printf("%f°\n", gyro.GetAngle());
			Wait(0.1);
			/*printf("x=%f y=%f z=%f\n", m_stick1.GetX(), m_stick1.GetY(), m_stick1.GetZ());
			Wait(0.1);*/
		}
	}
    void Autonomous()
    {
    	if (IsAutonomous() && IsEnabled())
    	{
    		printf("AUTO\n");
    		gyro.Reset();
    		while(ultrasonic.GetVoltage() * 102.3 > 35)
    		{

    		}
    	}
    }
	void OperatorControl() {
		printf("teleop\n");
		gyro.Calibrate();
		while (IsOperatorControl() && IsEnabled())
		{
			compensate = m_stick1.GetZ() * 0.3;
			if(fabs(m_stick1.GetX()) < 0.3 && fabs(m_stick1.GetY()) < 0.3)
			{
				MLeftFront.Set(m_stick1.GetZ() * 0.3);
				MRightRear.Set(m_stick1.GetZ() * 0.3);
				MRightFront.Set(m_stick1.GetZ() * 0.3);
				MLeftRear.Set(m_stick1.GetZ() * 0.3);
			}
			else
			{
				if(m_stick1.GetX() == 0)
				{
					MLeftFront.Set(m_stick1.GetY() * -0.8);
					MRightRear.Set(m_stick1.GetY() * 0.8);
					MRightFront.Set(m_stick1.GetY() * 0.8);
					MLeftRear.Set(m_stick1.GetY() * -0.8);
				}
				if(m_stick1.GetY() == 0)
				{
					MLeftFront.Set(m_stick1.GetX() * 0.8);
					MRightRear.Set(m_stick1.GetX() * -0.8);
					MRightFront.Set(m_stick1.GetX() * 0.8);
					MLeftRear.Set(m_stick1.GetX() * -0.8);
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
				{
					MLeftFront.Set(((256 + 340 * m_stick1.GetX())/255)*0.8 + compensate);
					MRightRear.Set(((256 + 340 * m_stick1.GetX())/255)*-0.8 + compensate);
					MRightFront.Set(m_stick1.GetY()*0.8 + compensate);
					MLeftRear.Set(m_stick1.GetY()*-0.8 + compensate);
					//printf("You are in the 8th quadrant\n");
					//printf("speed of LFRR=%f\n", (256 + 340 * m_stick1.GetX())/255);
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
				{
					MLeftFront.Set(((-256 - 340 * m_stick1.GetY())/255)*0.8 + compensate);
					MRightRear.Set(((-256 - 340 * m_stick1.GetY())/255)*-0.8 + compensate);
					MRightFront.Set(m_stick1.GetX() * 0.8 + compensate);
					MLeftRear.Set(m_stick1.GetX() * -0.8 + compensate);
					//printf("You are in the 7th quadrant\n");
					//printf("speed of LFRR=%f\n", (-256 - 340 * m_stick1.GetY())/255);
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
				{
					MLeftFront.Set(m_stick1.GetY() * -0.8 + compensate);
					MRightRear.Set(m_stick1.GetY() * 0.8 + compensate);
					MRightFront.Set(((256 + 320 * m_stick1.GetX())/-255)*-0.8 + compensate);
					MLeftRear.Set(((256 + 320 * m_stick1.GetX())/-255)*0.8 + compensate);
					//printf("speed of RFLR=%f\n", (256 + 320 * m_stick1.GetX())/-255);
					//printf("You are in the 5th quadrant\n");
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
				{
					MLeftFront.Set(m_stick1.GetX() * 0.8 + compensate);
					MRightRear.Set(m_stick1.GetX() * -0.8 + compensate);
					MRightFront.Set(((-256 + 320 * m_stick1.GetY())/-255)*-0.8 + compensate);
					MLeftRear.Set(((-256 + 320 * m_stick1.GetY())/-255)*0.8 + compensate);
					//printf("speed of RFLR=%f\n", (-256 + 320 * m_stick1.GetY())/-255);
					//printf("You are in the 6th quadrant\n");
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
				{
					MLeftFront.Set(m_stick1.GetY() * -0.8 + compensate);
					MRightRear.Set(m_stick1.GetY() * 0.8 + compensate);
					MRightFront.Set(((256 - 330 * m_stick1.GetX())/255)*-0.8 + compensate);
					MLeftRear.Set(((256 - 330 * m_stick1.GetX())/255)*0.8 + compensate);
					//printf("speed of RFLR=%f\n", (256 - 340 * m_stick1.GetX())/255);
					//printf("You are in the 1th quadrant\n");
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
				{
					MLeftFront.Set(m_stick1.GetX()*0.8 + compensate);
					MRightRear.Set(m_stick1.GetX()*-0.8 + compensate);
					MRightFront.Set(((256 + 330 * m_stick1.GetY())/-255)*-0.8 + compensate);
					MLeftRear.Set(((256 + 330 * m_stick1.GetY())/-255)*0.8 + compensate);
					//printf("speed of RFLR=%f\n", (256 + 330 * m_stick1.GetY())/-255);
					//printf("You are in the 2th quadrant\n");
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
				{
					MLeftFront.Set(((-256 + 340 * m_stick1.GetX())/255)*0.8 + compensate);
					MRightRear.Set(((-256 + 340 * m_stick1.GetX())/255)*-0.8 + compensate);
					MRightFront.Set(m_stick1.GetY() * 0.8 + compensate);
					MLeftRear.Set(m_stick1.GetY() * -0.8 + compensate);
					//printf("speed of LFRR=%f\n", (-256 + 340 * m_stick1.GetX())/255);
					//printf("You are in the 4th quadrant\n");
				}
				if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
				{
					MLeftFront.Set(((256 - 340 * m_stick1.GetY())/255)*0.8 + compensate);
					MRightRear.Set(((256 - 340 * m_stick1.GetY())/255)*-0.8 + compensate);
					MRightFront.Set(m_stick1.GetX() * 0.8 + compensate);
					MLeftRear.Set(m_stick1.GetX() * -0.8 + compensate);
					//printf("speed of LFRR=%f\n", (256 - 340 * m_stick1.GetY())/255);
					//printf("You are in the 3th quadrant\n");
				}
			}
			/*MLeftFront.Set(m_stick1.GetY() * 0.3);
			MRightRear.Set(m_stick1.GetY() * -0.3);
			MRightFront.Set(m_stick1.GetY() * -0.3);
			MLeftRear.Set(m_stick1.GetY() * 0.3);*/
			printf("angle=%f\n", gyro.GetAngle());

			Wait(0.05);
		}
	}
};
START_ROBOT_CLASS(Robot)
