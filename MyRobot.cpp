/**
 * This is team 2823's C++ program for the 2013 season. (Ultimate Ascent)
 */ 

#include "WPILib.h"
#include "Timer.h"
//#include "Vision2823.h"
#include "PIDJaguar.h"

#define WHEELSPEED 300
#define LOWERTHRESHOLD (WHEELSPEED-10)
#define UPPERTHRESHOLD (WHEELSPEED+10)
#define MINIMUMSPEED (WHEELSPEED-50)
class RobotDemo : public SimpleRobot
{
	RobotDrive DriveWheels;
	PIDJaguar Shooter;
	Relay Hurricane;
	Relay ShooterAngle;
	DigitalInput ShooterAngleUp;
	DigitalInput ShooterAngleDown;
	Joystick Gamepad;
	int HurricaneControl;
	bool Shooting;
	Timer TimerDown;
	Timer TimerUp;
	DigitalInput sensor1; //Magnetic Counter
	Encoder shootEncoder;
	DigitalInput HurricaneSwitch; //Hurricane ON/OFF
	PIDController ShooterPID;
	double StartTime;
	int StartCount;
	bool ShooterToggle;
	bool TargetLock;
	//Vision2823 vision;
	double AngleTime;
	
public:
	RobotDemo(void):
		DriveWheels(1, 2, 3, 4),
		Shooter(5),
		Hurricane(3),
		ShooterAngle(2),
		ShooterAngleUp(3),
		ShooterAngleDown(4),
		Gamepad(1),
		HurricaneControl(0),
		Shooting(false),
		TimerDown(),
		TimerUp(),
		sensor1(1),
		shootEncoder(sensor1, sensor1, false, Encoder::k1X),
		HurricaneSwitch(2),
		ShooterPID(-0.001, 0.0000, -0.0001, &shootEncoder, &Shooter),
		ShooterToggle(false),
		TargetLock(false),
		//vision(0.25),
		AngleTime(0.0)
	{
		//vision.Start();
		DriveWheels.SetExpiration(0.25);
		Shooter.SetExpiration(0.25);
		Shooter.SetEncoder(&shootEncoder);
		shootEncoder.SetPIDSourceParameter(Encoder::kRate);
		shootEncoder.Start();
		ShooterPID.SetInputRange(0.0, 350.0);
		ShooterPID.SetSetpoint(WHEELSPEED*1.0);
		ShooterPID.SetAbsoluteTolerance(5.0);
	}
	
	~RobotDemo() //Failsafe for stupid FRC people
	{
		//vision.Stop();
	}

	void AutoShoot(double howlong)
	{
		StartShooting();
		while (!UpdateShooting() && IsAutonomous() && IsEnabled()) //Only called by autonomous now
		{
			Wait(0.05);
		}
		StopShooting();
	}

	void Autonomous(void)
	{
		int PIDGoodCount = 0;
		int ShotsTaken = 0;
		DriveWheels.SetSafetyEnabled(false);
		Shooter.SetSafetyEnabled(false);
		ShooterPID.Enable();
		while (IsAutonomous() && IsEnabled() && ShotsTaken < 8)
		{
			if (shootEncoder.GetRate() >= LOWERTHRESHOLD && shootEncoder.GetRate() <= UPPERTHRESHOLD)
			{
				PIDGoodCount ++;
				printf ("%d\n",PIDGoodCount);
			}	
			else if (shootEncoder.GetRate() < MINIMUMSPEED)
			{
				PIDGoodCount=0;
			}
			if (PIDGoodCount>=5)
			{
				AutoShoot(2.0);
				PIDGoodCount=0;
				ShotsTaken ++;
			}
			Wait (0.05);
		}
		ShooterPID.Disable();
		Shooter.Set(0.0);
	}

	void OperatorControl(void)
	{
		DriveWheels.SetSafetyEnabled(true);
		Shooter.SetSafetyEnabled(true);
		bool PIDStarted = false;
		bool manualShooter = false;
		int PIDGoodCount=0;
		double lastspeed=-1;
		//bool DoAutoAim = true;
		
		bool lastHurricaneSwitch=!HurricaneSwitch.Get();
		bool lastShooterUp=!ShooterAngleUp.Get();
		bool lastShooterDown=!ShooterAngleDown.Get();
		
		//NetworkTable *distanceTable = NetworkTable::GetTable("SmartDashboard");

		bool button5DownPriorLoop = false;
		bool button5Down = false;
		StopShooting();
		while (IsOperatorControl() && IsEnabled())
		{
			Shooter.Feed();
			if (Gamepad.GetY() > 0.17 || Gamepad.GetY() < -0.17)
								{
									DriveWheels.TankDrive(Gamepad.GetY(), Gamepad.GetTwist());
								}
								else
								{
									DriveWheels.TankDrive(0.0, 0.0);
								}
	//		printf ("%g", Gamepad.GetX());
	//		printf ("%g\n", Gamepad.GetY());
			if (HurricaneSwitch.Get()!=lastHurricaneSwitch)
			{
				printf("Limit of the Hurricane Switch: %d\n", HurricaneSwitch.Get());
				lastHurricaneSwitch = HurricaneSwitch.Get();
			}
			if (ShooterAngleDown.Get()!=lastShooterDown)
			{
				printf("Limit of the ShooterAngleDown Switch: %d\n", ShooterAngleDown.Get());
				lastShooterDown = ShooterAngleDown.Get();
			}
			if (ShooterAngleUp.Get()!=lastShooterUp)
			{
				printf("Limit of the ShooterAngleUp Switch: %d\n", ShooterAngleUp.Get());
				lastShooterUp = ShooterAngleUp.Get();
			}
			if (Gamepad.GetRawButton(4))
			{
				TimerDown.Start();
				AngleTime=5.61;
			}
			if (Gamepad.GetRawButton(3))
			{
				TimerUp.Start();
				AngleTime=5.5;
			}
#ifdef twobutton
			if (Gamepad.GetRawButton(2)==1)
			{
				Hurricane.Set(Relay::kReverse);
			}
			else
			{
				Hurricane.Set(Relay::kOff);
			}
#endif
			if (Gamepad.GetRawAxis(6) == -1 || (TimerDown.Get() > 0 && TimerDown.HasPeriodPassed(AngleTime) == false))
			{
				AngleMove(-1);
			}
			else if (Gamepad.GetRawAxis(6) == 1 || (TimerUp.Get() > 0 && TimerUp.HasPeriodPassed(AngleTime) == false)) 
			{
				AngleMove(1);
			}
			else
			{
				AngleMove(0);
				TimerUp.Stop();
				TimerDown.Stop();
				TimerUp.Reset();
				TimerDown.Reset();
			}
			if (Gamepad.GetRawAxis(6)==1 || Gamepad.GetRawAxis(6)==-1)
			{
				AngleTime=0.0;
			}	
			if (shootEncoder.GetRate() >= LOWERTHRESHOLD && shootEncoder.GetRate() <= UPPERTHRESHOLD)
			{
				PIDGoodCount ++;
			}	
			else if (shootEncoder.GetRate() < MINIMUMSPEED)
			{
				PIDGoodCount=0;
			}
				
			if (Gamepad.GetRawButton(8) || (Gamepad.GetRawButton(6) && PIDGoodCount>=5))
			{
				StartShooting();
				PIDGoodCount=0;
			}
			
			if (Gamepad.GetRawButton(7))
			{
				Shooter.Set(-1);
				manualShooter = true;
				if (ShooterToggle)
					ShooterToggle = false;
			}
			else
			{
				if (manualShooter)
					Shooter.Set(0);
				manualShooter = false;
			}

			button5Down = Gamepad.GetRawButton(5);
			if (button5Down && !button5DownPriorLoop)
			{
				ShooterToggle = !ShooterToggle;
			}
			button5DownPriorLoop = button5Down;
			
			if (shootEncoder.GetRate() != lastspeed)
			{    
				//distanceTable->PutNumber("speed",shootEncoder.GetRate());
				lastspeed = shootEncoder.GetRate();
				printf ("%g\n", lastspeed);
			}
			if (ShooterToggle)
			{
				if (! PIDStarted)
				{
					ShooterPID.Enable();
				}
				PIDStarted = true;
			}
			else
			{
				if (PIDStarted)
				{
					ShooterPID.Disable();
					Shooter.Set(0.0);
				}
				PIDStarted = false;
			}

#ifdef visionon
			if (Gamepad.GetRawButton(1)&& vision.isHighGoal && DoAutoAim)
			{
					AutoAim(vision.highY, PerfectY(vision.highWidth));
					DoAutoAim=false;
			}

#endif
			
			UpdateShooting();
			//printf ("%d\n",PIDGoodCount);

#ifdef visionon
			if (vision.Updated())
			{
				if (vision.isHighGoal)
					distanceTable->PutNumber("HighGoalNumber",1);
				else
					distanceTable->PutNumber("HighGoalNumber",0);

				if (vision.isHighGoal)
				{
					distanceTable->PutNumber("highD", vision.highDistance2);
					distanceTable->PutNumber("highX", vision.highX);
					distanceTable->PutNumber("highY", vision.highY);
					distanceTable->PutNumber("highWidth", vision.highWidth);
					distanceTable->PutNumber("highHeight", vision.highHeight);
					distanceTable->PutNumber("PerfectY", PerfectY(vision.highWidth));
				}
				vision.Clear();
				DoAutoAim=true;
			}
#endif
			Wait(0.01);
		}
		ShooterPID.Disable();
		Hurricane.Set(Relay::kOff);
		Shooter.Set(0.0);
	}
	int AngleMove(int Direction)
	{
		if (Direction == 1 && ShooterAngleDown.Get()==1)
		{
			ShooterAngle.Set(Relay::kForward);
		}
		else if (Direction == -1 && ShooterAngleUp.Get()==1)
		{
			ShooterAngle.Set(Relay::kReverse);
		}
		else
		{
			ShooterAngle.Set(Relay::kOff);
		}
		return (Direction);
	}
	
	int PerfectY(int w)
	{
		/*
		X	Y	Height	Width	Distance
		147	209	25	77	26.802
		145	190	34	106	19.470
		165	173	39	125	16.643
		171	154	46	155	13.315
		161	142	51	189	10.919
		 */

		double ww = 1.0 * w;
		double ret;

		ret = (.00004593677 * ww) * (ww * ww);

		ret -= .01593994 * (ww * ww);

		ret += 1.05940095 * ww;

		ret += 201.138;

		return (int) ret;

	}

	void AutoAim(int CurrentY, int Py)
	{
		double ANGLESPEED = 40.0;
		double ytime = (Py-CurrentY) / ANGLESPEED;
		if ((Py<=CurrentY+2) && (Py>=CurrentY-2))
		{
			return ;	
		}
		if (ytime > 0)
		{
			TimerUp.Start();
			TimerDown.Stop();
			TimerDown.Reset();
			AngleTime=ytime;
		}
		if (ytime < 0)
		{
			TimerDown.Start();
			AngleTime=-1*ytime;
			TimerUp.Stop();
			TimerUp.Reset();
		}
	}
	
	void StartShooting()
	{
		if (Shooting == false)
		{
			HurricaneControl = 1;
		}
		Shooting = true;
	}
	
	bool UpdateShooting()
	{
		bool ShotComplete = false;
		if (HurricaneControl == 1)
		{
			Hurricane.Set(Relay::kReverse);
			printf ("Hurricane On\n");
		}
		if (HurricaneSwitch.Get() == 0 && HurricaneControl == 1)
		{
			HurricaneControl = 2;
		}
		if (HurricaneSwitch.Get() == 1 && HurricaneControl == 1)
		{
			HurricaneControl = 3;
		}
		if (HurricaneControl == 2 && HurricaneSwitch.Get() == 1)
		{
			HurricaneControl = 3;
		}
		if (HurricaneControl == 3 && HurricaneSwitch.Get() == 0)
		{
			StopShooting();
			ShotComplete = true;
			printf("shot complete\n");
		}
		return ShotComplete;
	}
	
	void StopShooting()
	{
		HurricaneControl = 0;
		Shooting = false;
		Hurricane.Set(Relay::kOff);
		printf ("Hurricane Off\n");
	}
	
	void Test()
	{
		DriveWheels.SetSafetyEnabled(false);
		Shooter.SetSafetyEnabled(false);

		bool lastHurricaneSwitch=!HurricaneSwitch.Get();
		bool lastShooterUp=!ShooterAngleUp.Get();
		bool lastShooterDown=!ShooterAngleDown.Get();

		while (IsTest() && IsEnabled())
		{
			if (HurricaneSwitch.Get()!=lastHurricaneSwitch)
			{
				printf("Limit of the Hurricane Switch: %d\n", HurricaneSwitch.Get());
				lastHurricaneSwitch = HurricaneSwitch.Get();
			}
			if (ShooterAngleDown.Get()!=lastShooterDown)
			{
				printf("Limit of the ShooterAngleDown Switch: %d\n", ShooterAngleDown.Get());
				lastShooterDown = ShooterAngleDown.Get();
			}
			if (ShooterAngleUp.Get()!=lastShooterUp)
			{
				printf("Limit of the ShooterAngleUp Switch: %d\n", ShooterAngleUp.Get());
				lastShooterUp = ShooterAngleUp.Get();
			}
			Wait(.05);
			
		}
	}
};

START_ROBOT_CLASS(RobotDemo);
