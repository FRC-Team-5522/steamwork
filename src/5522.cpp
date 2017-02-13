#include <unistd.h>
#include "WPILib.h"
#include <stdio.h>
#include <sys/time.h>
#include <AnalogGyro.h>
#include <iostream>
#include <math.h>
#include <ADXRS450_Gyro.h>
#include <SerialPort.h>
class Robot : public SampleRobot
{
	public:std::shared_ptr<NetworkTable> table;
	Joystick m_stick1;
	Joystick m_stick2;
	DigitalInput di;
	ADXRS450_Gyro gyro;
	int bigshit;
	int smallshit;
	int oh;
	int smallerone;
	int biggerone;
	float last_dis;
	float compensate;
	float angle;
	Talon MLeftFront;
	Talon MLeftRear;
	Talon MRightFront;
	Talon MRightRear;
	Talon HANGGG;
	SerialPort ultrasonic;
	char ultrasonic_rx_buffer[10];
	int ultrasonic_rx_buffer_ptr;
	int ultrasonic_range;
	frc::DoubleSolenoid m_doubleSolenoid { 1, 2 };
	const double kUpdatePeriod = 0.005;
	const double valueToCMs = 0.3175;
	void getRange()
	{
		return;
		int num = ultrasonic.GetBytesReceived();
		while(num > 0)
		{
			num--;
			char ch = 0;
			ultrasonic.Read(&ch, 1);
			printf("%c", ch);
			if(ultrasonic_rx_buffer_ptr == -1)
			{
				if(ch == 'R')
				{
					memset(&ultrasonic_rx_buffer, 0, sizeof(ultrasonic_rx_buffer));
					ultrasonic_rx_buffer_ptr = 0;
				}
			}
			else
			{
				if(ch == 13)
				{
					ultrasonic_range = strtol(ultrasonic_rx_buffer, NULL, 10);
				}
				else
				{
					ultrasonic_rx_buffer[ultrasonic_rx_buffer_ptr] = ch;
					ultrasonic_rx_buffer_ptr++;
				}
			}
		}
	}
/*	void OLDautoTargeting()
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
		}
	}*/
	void TURTLEmode()
	{
		if(m_stick1.GetX() == 0)//Calibrate the power input!!!!!!!!!!!
		{
			MLeftFront.Set(m_stick1.GetY() * -0.345 + compensate);
			MRightRear.Set(m_stick1.GetY() * 0.3 + compensate);
			MRightFront.Set(m_stick1.GetY() * 0.3 + compensate);
			MLeftRear.Set(m_stick1.GetY() * -0.345 + compensate);
		}
		if(m_stick1.GetY() == 0)
		{
			MLeftFront.Set(m_stick1.GetX() * 0.3 + compensate);
			MRightRear.Set(m_stick1.GetX() * -0.3 + compensate);
			MRightFront.Set(m_stick1.GetX() * 0.3 + compensate);
			MLeftRear.Set(m_stick1.GetX() * -0.3 + compensate);
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(((256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			MRightRear.Set(((256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MRightFront.Set(pow(squaOFxny,0.5)*-0.3 + compensate);
			MLeftRear.Set(pow(squaOFxny,0.5)*0.3 + compensate);
			//printf("You are in the 8th quadrant\n");
			//printf("speed of LFRR=%f\n", (256 + 340 * m_stick1.GetX())/255);
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(((-256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			MRightRear.Set(((-256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MRightFront.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			MLeftRear.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			//printf("You are in the 7th quadrant\n");
			//printf("speed of LFRR=%f\n", (-256 - 340 * m_stick1.GetY())/255);
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			MRightRear.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MRightFront.Set(((256 + 320 * m_stick1.GetX())/-255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MLeftRear.Set(((256 + 320 * m_stick1.GetX())/-255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			//printf("speed of RFLR=%f\n", (256 + 320 * m_stick1.GetX())/-255);
			//printf("You are in the 5th quadrant\n");
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			MRightRear.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MRightFront.Set(((-256 + 320 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MLeftRear.Set(((-256 + 320 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			//printf("speed of RFLR=%f\n", (-256 + 320 * m_stick1.GetY())/-255);
			//printf("You are in the 6th quadrant\n");
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MRightRear.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			MRightFront.Set(((256 - 330 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MLeftRear.Set(((256 - 330 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			//printf("speed of RFLR=%f\n", (256 - 340 * m_stick1.GetX())/255);
			//printf("You are in the 1th quadrant\n");
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MRightRear.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			MRightFront.Set(((256 + 330 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MLeftRear.Set(((256 + 330 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			//printf("speed of RFLR=%f\n", (256 + 330 * m_stick1.GetY())/-255);
			//printf("You are in the 2th quadrant\n");
		}
			if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(((-256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			MRightRear.Set(((-256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MRightFront.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MLeftRear.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			//printf("speed of LFRR=%f\n", (-256 + 340 * m_stick1.GetX())/255);
			//printf("You are in the 4th quadrant\n");
		}
		if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
		{
			double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
			MLeftFront.Set(((256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*0.3 + compensate);
			MRightRear.Set(((256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*-0.3 + compensate);
			MRightFront.Set(pow(squaOFxny,0.5) * 0.3 + compensate);
			MLeftRear.Set(pow(squaOFxny,0.5) * -0.3 + compensate);
			//printf("speed of LFRR=%f\n", (256 - 340 * m_stick1.GetY())/255);
			//printf("You are in the 3th quadrant\n");
		}
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
		HANGGG(5),
		gyro(),
		di(1),
		ultrasonic(9600,
				   frc::SerialPort::kOnboard,
				   8,
				   frc::SerialPort::kParity_None,
				   frc::SerialPort::kStopBits_One)
	{
	}
	void RobotInit()
		{
			/*if (fork() == 0)
			{
				system("/bin/sh /home/lvuser/start-grip.sh");
				return;
				// #/bin/sh
				// ps | grep grip | grep -v grep | awk '{print $1}' | xargs kill
				// /usr/local/frc/JRE//bin/java -Xmx50m -XX:-OmitStackTraceInFastThrow -XX:+HeapDumpOnOutOfMemoryError -jar '/home/lvuser/grip.jar' '/home/lvuser/project.grip' > /dev/null
			}*/
			table = NetworkTable::GetTable("/GRIP/Shit");
			compensate = 0;
			last_dis = 0;
			oh = 0;
			angle = 0;
			smallerone = 0;
			/*ultrasonic.SetReadBufferSize(100);
			ultrasonic.SetWriteBufferSize(100);
			ultrasonic.Reset();
			memset(&ultrasonic_rx_buffer, 0, sizeof(ultrasonic_rx_buffer));
			ultrasonic_rx_buffer_ptr = -1;
			ultrasonic_range = -1;*/
			gyro.Calibrate();
			printf("init\n");
		}
	void Test()
	{
		gyro.Calibrate();//calibration needs 3-4sec, so put this line into RobotInit.
		//Warning!!!算了用中文写 注意！！！因为calibration的时候机器人绝对绝对！！！绝对！！！不能动！！！所以要确保机器人稳稳的不动的时候再开电
		//开店之后Calibration之后 比赛那边会调试很久很久 累积误差会增大 所以在auto开始的时候先reset一下！！！谨记！！！！！！！！
		while(IsTest() && IsEnabled())
		{
			getRange();
			printf("%d mm\n", ultrasonic_range);
			//Wait(0.1);
			//printf("%f°\n", gyro.GetAngle());
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
    		getRange();
    		//printf("angle=%f°\n", gyro.GetAngle());
    		while(oh < 15000)//1s
    		{
    			angle = gyro.GetAngle();
    			if(angle < 2 && angle > -2)
    			{
    				MLeftFront.Set(0.5);
    				MRightRear.Set(-0.54);
    				MRightFront.Set(-0.54);
    				MLeftRear.Set(0.5);
    				//power needs to be calibrate, if u wanna go straight, simply input the same power on two side wont work.
    			}
    			if(angle > 2)
    			{
    				MLeftFront.Set(0);
    				MRightRear.Set(-0.5);
    				MRightFront.Set(-0.5);
    				MLeftRear.Set(0);
    			}
    			if(angle < -2)
    			{
    				MLeftFront.Set(0.5);
    				MRightRear.Set(0);
    				MRightFront.Set(0);
    				MLeftRear.Set(0.5);
    			}
    			oh++;
    			getRange();
    		}
    		while(oh < 48000)//1s
    		{
    			angle = gyro.GetAngle();
    			if(angle < 2 && angle > -2)
    			{
    				MLeftFront.Set(0.4);
    				MRightRear.Set(-0.44);
    				MRightFront.Set(-0.44);
    				MLeftRear.Set(0.4);
    				//power needs to be calibrate, if u wanna go straight, simply input the same power on two side wont work.
    			}
    			if(angle > 2)
    			{
    				MLeftFront.Set(0);
    				MRightRear.Set(-0.4);
    				MRightFront.Set(-0.4);
    				MLeftRear.Set(0);
    			}
    			if(angle < -2)
    			{
    				MLeftFront.Set(0.4);
    				MRightRear.Set(0);
    				MRightFront.Set(0);
    				MLeftRear.Set(0.4);
    			}
    			oh++;
    			getRange();
    		}
    		printf("step1 angle=%f\n", angle);
    		allStop();
    		Wait(0.2);
    		angle = gyro.GetAngle();
    		while(angle < -0.5 || angle > 0.3)//1.5s
    		{
    			double angleD = gyro.GetAngle();
    			double FABSangleD = fabs(angleD);
    			double anglecomp = pow(FABSangleD,3);
    			if(angle > -0.1)//The angle will be decided by the actual test result
    			{
    				MLeftFront.Set(-0.1 + anglecomp * -0.000015);
    				MRightRear.Set(-0.1 + anglecomp * -0.000015);
    				MRightFront.Set(-0.1 + anglecomp * -0.000015);
    				MLeftRear.Set(-0.1 + anglecomp * -0.000015);
    			}
    			else
    			{
    				MLeftFront.Set(0.1 + anglecomp * 0.00001);
    				MRightRear.Set(0.1 + anglecomp * 0.00001);
    				MRightFront.Set(0.1 + anglecomp * 0.00001);
    				MLeftRear.Set(0.1 + anglecomp * 0.00001);
    			}
    			Wait(0.1);
    			angle = gyro.GetAngle();
    		}
    		allStop();
    		Wait(0.2);
    		printf("step2 angle=%f\n", angle);
    		while(angle < -0.5 || angle > 0.3)//1.5s
       		{
       			double angleD = gyro.GetAngle();
       			double FABSangleD = fabs(angleD);
       			double anglecomp = pow(FABSangleD,3);
       			if(angle > -0.1)//The angle will be decided by the actual test result
       			{
       				MLeftFront.Set(-0.1 + anglecomp * -0.000015);
       				MRightRear.Set(-0.1 + anglecomp * -0.000015);
       				MRightFront.Set(-0.1 + anglecomp * -0.000015);
       				MLeftRear.Set(-0.1 + anglecomp * -0.000015);
       			}
       			else
       			{
       				MLeftFront.Set(0.1 + anglecomp * 0.00001);
       				MRightRear.Set(0.1 + anglecomp * 0.00001);
       				MRightFront.Set(0.1 + anglecomp * 0.00001);
       				MLeftRear.Set(0.1 + anglecomp * 0.00001);
      			}
       			Wait(0.1);
       			angle = gyro.GetAngle();
       		}
    		allStop();
    		Wait(0.2);
    		printf("step3 angle=%f\n", angle);
    		/*std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());
    		std::vector<double> centerXs = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
    		std::vector<double> centerYs = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
    		if(centerXs[1] > centerXs[2])
    		{
    			smallerone = 2;
    			biggerone = 1;
    		}
    		else
    		{
    			smallerone = 1;
    			biggerone = 2;
    		}
    		//This constant need to be timed by a specific constant
    		float time = fabs(centerXs[smallerone] + ((centerXs[biggerone] - centerXs[smallerone]) / 2) - 320) * 0.3;//times some constant
    		if((centerXs[smallerone] + ((centerXs[biggerone] - centerXs[smallerone]) / 2)) < 320)//2s
    		{
    			MLeftFront.Set(0.25);
    			MRightRear.Set(-0.25);
    			MRightFront.Set(0.25);
    			MLeftRear.Set(-0.25);
    			Wait(time);
    		}
    		else	//Test this part individually
    		{
    			MLeftFront.Set(-0.25);
      			MRightRear.Set(0.25);
       			MRightFront.Set(-0.25);
       			MLeftRear.Set(0.25);
       			Wait(time);
       		}*/
    		allStop();
    		Wait(0.2);
    		printf("step4 angle=%f\n", angle);
    		//Turn left or turn right first will be decided by the actual test result
    	    MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	    MRightFront.Set(0);
    	    MLeftRear.Set(0.25);
    	    Wait(0.5);
    	    MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	   	MRightFront.Set(0);
    	   	MLeftRear.Set(0.25);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	   	MRightFront.Set(0);
    	   	MLeftRear.Set(0.25);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	   	MRightFront.Set(0);
    	   	MLeftRear.Set(0.25);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	   	MRightFront.Set(0);
    	   	MLeftRear.Set(0.25);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);MLeftFront.Set(0.25);
    	   	MRightRear.Set(0);
    	   	MRightFront.Set(0);
    	   	MLeftRear.Set(0.25);
    	   	Wait(0.5);
    	   	MLeftFront.Set(0);
    	   	MRightRear.Set(-0.25);
    	   	MRightFront.Set(-0.25);
    	   	MLeftRear.Set(0);
    	   	Wait(0.5);
    	   	allStop();
    	   	Wait(1);
    	}
    }
	void OperatorControl()
	{
		printf("teleop\n");
		//gyro.Calibrate();
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
				if(m_stick1.GetRawButton(5))
				{
					TURTLEmode();
				}
				else
				{
					if(m_stick1.GetX() == 0)//Calibrate the power input!!!!!!!!!!!!!!!!!!!
					{
						MLeftFront.Set(m_stick1.GetY() * -0.96 + compensate);
						MRightRear.Set(m_stick1.GetY() * 0.93 + compensate);
						MRightFront.Set(m_stick1.GetY() * 0.93 + compensate);
						MLeftRear.Set(m_stick1.GetY() * -0.96 + compensate);
					}
					if(m_stick1.GetY() == 0)
					{
						MLeftFront.Set(m_stick1.GetX() * 0.95 + compensate);
						MRightRear.Set(m_stick1.GetX() * -0.95 + compensate);
						MRightFront.Set(m_stick1.GetX() * 0.95 + compensate);
						MLeftRear.Set(m_stick1.GetX() * -0.95 + compensate);
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(((256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						MRightRear.Set(((256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MRightFront.Set(pow(squaOFxny,0.5)*-0.95 + compensate);
						MLeftRear.Set(pow(squaOFxny,0.5)*0.95 + compensate);
						//printf("You are in the 8th quadrant\n");
						//printf("speed of LFRR=%f\n", (256 + 340 * m_stick1.GetX())/255);
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() < 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(((-256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						MRightRear.Set(((-256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MRightFront.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						MLeftRear.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						//printf("You are in the 7th quadrant\n");
						//printf("speed of LFRR=%f\n", (-256 - 340 * m_stick1.GetY())/255);
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						MRightRear.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MRightFront.Set(((256 + 320 * m_stick1.GetX())/-255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MLeftRear.Set(((256 + 320 * m_stick1.GetX())/-255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						//printf("speed of RFLR=%f\n", (256 + 320 * m_stick1.GetX())/-255);
						//printf("You are in the 5th quadrant\n");
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() < 0 && m_stick1.GetY() > 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						MRightRear.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MRightFront.Set(((-256 + 320 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MLeftRear.Set(((-256 + 320 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						//printf("speed of RFLR=%f\n", (-256 + 320 * m_stick1.GetY())/-255);
						//printf("You are in the 6th quadrant\n");
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MRightRear.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						MRightFront.Set(((256 - 330 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MLeftRear.Set(((256 - 330 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						//printf("speed of RFLR=%f\n", (256 - 340 * m_stick1.GetX())/255);
						//printf("You are in the 1th quadrant\n");
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() < 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MRightRear.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						MRightFront.Set(((256 + 330 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MLeftRear.Set(((256 + 330 * m_stick1.GetY())/-255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						//printf("speed of RFLR=%f\n", (256 + 330 * m_stick1.GetY())/-255);
						//printf("You are in the 2th quadrant\n");
					}
						if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) >= 0 && fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) <= 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(((-256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						MRightRear.Set(((-256 + 340 * m_stick1.GetX())/255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MRightFront.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MLeftRear.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						//printf("speed of LFRR=%f\n", (-256 + 340 * m_stick1.GetX())/255);
						//printf("You are in the 4th quadrant\n");
					}
					if(fabs(m_stick1.GetX())/fabs(m_stick1.GetY()) > 1 && m_stick1.GetX() > 0 && m_stick1.GetY() > 0)
					{
						double squaOFxny = pow(m_stick1.GetX(),2) + pow(m_stick1.GetY(),2);
						MLeftFront.Set(((256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*0.95 + compensate);
						MRightRear.Set(((256 - 340 * m_stick1.GetY())/255)*(pow(squaOFxny,0.5))*-0.95 + compensate);
						MRightFront.Set(pow(squaOFxny,0.5) * 0.95 + compensate);
						MLeftRear.Set(pow(squaOFxny,0.5) * -0.95 + compensate);
						//printf("speed of LFRR=%f\n", (256 - 340 * m_stick1.GetY())/255);
						//printf("You are in the 3th quadrant\n");
					}
				}
			}
			/*MLeftFront.Set(m_stick1.GetY() * 0.3);
			MRightRear.Set(m_stick1.GetY() * -0.3);
			MRightFront.Set(m_stick1.GetY() * -0.3);
			MLeftRear.Set(m_stick1.GetY() * 0.3);*/
			//printf("angle=%f\n", gyro.GetAngle());
			/*float v = ultrasonic.GetVoltage();
			if(abs(round((v - last_dis))*100) > 5)
			{
				last_dis = v;
				printf("v=%f dis = %f cm\n", v, (v / ( 5.0 / 1024.0) * 5.0 / 10.0));
			}*/
			if(m_stick1.GetRawButton(4))
			{
				HANGGG.Set(1);
			}
			else
			{
				HANGGG.Set(0);
			}
			if (m_stick1.GetRawButton(1))
			{
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
			}
			else if (m_stick1.GetRawButton(2))
			{
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
			}
			else
			{
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
			}
			Wait(0.05);
		}

	}
};
START_ROBOT_CLASS(Robot)
