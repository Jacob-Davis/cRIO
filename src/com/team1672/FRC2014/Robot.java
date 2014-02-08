/*
    SARCS - Semi-Automatic Robot Control System
    v1.1 'junefire'

    This code is to be run on the cRIO.

    Copyright ©2014 Tommy Bohde, Jeff Meli, and Ian Rahimi.

    SARCS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SARCS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SARCS. If not, see <http://www.gnu.org/licenses/>.
*/

package com.team1672.FRC2014;

/**
 * Gets the goodies.
 */
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SimpleRobot
{
	// General functionality variables
  private long ticks;
  private long lastReverseTime;
	private long lastSwitchTime;
	private long lastSSTime;
  private boolean isCompressorOn;
  private boolean isInverted;
	private boolean isCameraUp;
	private boolean isSensitiveAtSlowSpeeds;
	private final DigitalInput pressureSwitch;

  /* Buttons */
  public final int FIRE_BUTTON = 1;
  public final int LIFT_UP_BUTTON = 3;
  public final int LIFT_DOWN_BUTTON = 2;
  public final int CAMERA_TOGGLE = 3;
  public final int COMPRESSOR_BUTTON = 5;
	public final int SPEED_SENSITIVITY_TOGGLE = 11;
  /* Joystick USB Ports */ /* XXX: This may vary on different computers at different times */
  public final int kLeftJoystick = 1;
  public final int kRightJoystick = 2;
	public final double LIFT_SPEED = 0.25;
	public final double[] cameraAngle = {0.7, 0.85};
  

  /* PWM Channels controlling motors */
  public final int[] kDrivetrain = {1, 2, 3, 4};
  public final int kLift = 5;
  public final int kCameraServo = 6; /* XXX: Remember to put a jumper next to this PWM channel */
  public final RobotDrive.MotorType[] motors = {RobotDrive.MotorType.kFrontLeft,
																								RobotDrive.MotorType.kFrontRight,
																								RobotDrive.MotorType.kRearLeft,
																								RobotDrive.MotorType.kRearRight};
	
	/* Relays (off of Digital Sidecar) */
  public final int kCompressor = 1;

  /* Digital IO (off of Digital Sidecar) */
  public final int kPressureSwitch = 13;

  /* Analog IO (off of cRIO module) */
  public final int kUltrasonic = 1;

  /* Solenoids (off of the cRIO NI 9472 module (the one with the LEDs on top)) */
  public final int[] kLeftSolenoid = {1, 2};
  public final int[] kRightSolenoid = {3, 4};
	
	// Vital functionality objects
	private RobotDrive drivetrain;
  private Jaguar lift;
  private Servo cameraServo;
	private Compressor compressor;
  private AnalogChannel ultrasonic;
  private DoubleSolenoid leftSolenoid, rightSolenoid;
	private Joystick leftStick, rightStick;
  
  public Robot()
	{
		//Initializes functionality variables
	  pressureSwitch = new DigitalInput(kPressureSwitch);
	  lastSwitchTime = 0;
	  lastReverseTime = 0;
	  isCompressorOn = false;
	  isInverted = false;
		isSensitiveAtSlowSpeeds = false;
    ticks = 1;

		//Sets up driving mechanisms (joysticks and drivetrain)
    leftStick = new Joystick(kLeftJoystick);
    rightStick = new Joystick(kRightJoystick);
    drivetrain = new RobotDrive(kDrivetrain[0],
                                kDrivetrain[1],
                                kDrivetrain[2],
                                kDrivetrain[3]);
	  invertMotors();
	  
		//Sets up lift mechanism motor
    lift = new Jaguar(kLift);
		
		//Sets up axis camera and servo motor
    cameraServo = new Servo(kCameraServo);
    cameraServo.set(cameraAngle[1]);
		isCameraUp = true;
		
		//Sets up solenoids, defaults to closed (off)
    leftSolenoid = new DoubleSolenoid(kLeftSolenoid[0], kLeftSolenoid[1]);
    leftSolenoid.set(DoubleSolenoid.Value.kOff);
    rightSolenoid = new DoubleSolenoid(kRightSolenoid[0], kRightSolenoid[1]);
    rightSolenoid.set(DoubleSolenoid.Value.kOff);

		//Sets up compressor
    compressor = new Compressor(2, 13, 2, 1);

		//Sets up ultrasonic sensors
    ultrasonic = new AnalogChannel(1);
	}

	public void autonomous() 
	{
	  /* TODO: Implement autonomous. */
		System.out.println("Autonomous mode currently has no purpose; switch to manual operation.");
	}

	public void operatorControl() 
	{
	  System.out.println("Driver operation enabled.");
		drivetrain.setSafetyEnabled(false);
		lift.setSafetyEnabled(false);
	
		while(this.isOperatorControl() && this.isEnabled()) 
		{
			drivetrain.tankDrive(leftStick, rightStick, isSensitiveAtSlowSpeeds);
     
		/* Z-axis is the throttle lever on the Logitech Attack 3 joystick;
     * it has a value on the interval [-1, 1], where -1 is physically 
		 * located at the top of the lever (near the positive sign)
	   * and 1 is at the bottom of the lever (near the negative sign).
		 * This feature is not currently implemented.
		 * 
     * double stickZLeft = 1 - ((leftStick.getZ() + 1) / 2);
		 * double stickZRight = 1 - ((rightStick.getZ() + 1) / 2);
		 * double liftSpeed = (stickZLeft + stickZRight) / 2;
		 */
			
			//Lift mechanism controls
			if(rightStick.getRawButton(LIFT_DOWN_BUTTON))
				lift.set(LIFT_SPEED);
			else if(rightStick.getRawButton(LIFT_UP_BUTTON))
				lift.set(-LIFT_SPEED);
			else
				lift.set(0);

			System.out.println("Pneumatic switch: " + pressureSwitch.get());
			System.out.println("Compressor: " + isCompressorOn);
			
			//Compressor controls (TEMPORARY)
		  if((leftStick.getRawButton(COMPRESSOR_BUTTON) || rightStick.getRawButton(COMPRESSOR_BUTTON)) && (System.currentTimeMillis() - lastSwitchTime) >= 250)
		  {
			  lastSwitchTime = System.currentTimeMillis();
				isCompressorOn = !isCompressorOn;
		  }
		  if(isCompressorOn)
			  compressor.setRelayValue(Relay.Value.kForward);
			else
				compressor.setRelayValue(Relay.Value.kOff);
		  if(compressor.getPressureSwitchValue())
			  compressor.start();
	    else
		    compressor.stop();
			
	    /**
			 * Solenoid controls
			 * NOTE: Swapping the wire plugs on the cRIO will reverse this functionality!
			 * Be careful!
			 */
		  if(rightStick.getRawButton(FIRE_BUTTON))
			{
				leftSolenoid.set(DoubleSolenoid.Value.kForward);
	      rightSolenoid.set(DoubleSolenoid.Value.kForward);
			}
			else if(leftStick.getRawButton(FIRE_BUTTON))
			{
				leftSolenoid.set(DoubleSolenoid.Value.kReverse);
			  rightSolenoid.set(DoubleSolenoid.Value.kReverse);
			}
	    else
			{
				leftSolenoid.set(DoubleSolenoid.Value.kOff);
	      rightSolenoid.set(DoubleSolenoid.Value.kOff);
			}
		
			//Camera control (up/down toggle)
			if(leftStick.getRawButton(CAMERA_TOGGLE)) 
				toggleCameraAngle();
			//Motor inversion control
		  if(rightStick.getRawButton(11) && (System.currentTimeMillis() - lastReverseTime) >= 250)
			  invertMotors();
			//Acceleration curve toggle (linear/exponential)
			if(rightStick.getRawButton(SPEED_SENSITIVITY_TOGGLE) && (System.currentTimeMillis() - lastSSTime) >= 250)
				toggleSpeedSensitivity();

			/* Periodic diagnostic messages */
			if(ticks % 100 == 0) 
			{
				System.out.println("Current tick: " + ticks);
				System.out.println("Analog channel 1: " + ultrasonic.getValue());
			}
	    ticks++;
		}
	}

	public void test() 
	{
		System.out.println("Test mode enabled. \n");
		System.out.println(drivetrain.getDescription() + ", " + drivetrain.toString());
		System.out.println(leftStick.toString());
		System.out.println(rightStick.toString());
		System.out.println(leftSolenoid.toString());
		System.out.println(rightSolenoid.toString());
	}

	public void disabled() 
	{
	//	compressor.setRelayValue(Relay.Value.kForward);
	//	compressor.start();
	//  while (!this.isEnabled()) {
	//    if(!compressor.getPressureSwitchValue()) {
	//      compressor.start();
	//    } else {
	//      compressor.stop();
	//    }
	//  }
	}
	
	/**
	 * Inverts the current state of the motors. If the motors are already inverted,
	 * they are returned to normal. Useful to change whether the picking-up side
	 * of the robot or the throwing side of the robot is the front.
	 */
	private void invertMotors()
	{
		lastReverseTime = System.currentTimeMillis();
		isInverted = !isInverted;
		for (int i = 0; i < 4; i++)
			drivetrain.setInvertedMotor(motors[i], isInverted);
	}
	
	/**
	 * Toggles the angle of the axis camera. The camera can either be up or down.
	 */
	private void toggleCameraAngle()
	{
		if(isCameraUp)
			cameraServo.set(cameraAngle[0]);
		else
			cameraServo.set(cameraAngle[1]);
		isCameraUp = !isCameraUp;
	}
	
	/**
	 * Toggles the acceleration properties of the drivetrain. If <code>isSensitiveAtSlowSpeeds</code>
	 * is <code>true</code>, the acceleration of the drivetrain is exponential, as opposed to linear.
	 */
	private void toggleSpeedSensitivity()
	{
		isSensitiveAtSlowSpeeds = !isSensitiveAtSlowSpeeds;
		drivetrain.tankDrive(leftStick, rightStick, isSensitiveAtSlowSpeeds);
	}
}
