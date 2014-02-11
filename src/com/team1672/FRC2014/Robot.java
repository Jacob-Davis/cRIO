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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Robot extends SimpleRobot
{
	// General functionality variables
	private int lcdLine;
	private long lastAngleTime;
  private long lastReverseTime;
	private long lastSSTime;
	private long lastSwitchTime;
  private boolean isCompressorOn;
  private boolean isInverted;
	private boolean isCameraUp;
	private boolean isSensitiveAtSlowSpeeds;
	private String[] lcdLines;
//	private Ultrasonic.Unit leftDistance, rightDistance;
	
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
	
	// Other constants
	public final int[] PING_CHANNELS = {1, 3};
	public final int[] ECHO_CHANNELS = {2, 4};
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
	
	// Lines on the Driver Station LCD (User Messages Section)
	public final DriverStationLCD.Line[] line = {DriverStationLCD.Line.kUser1,
																							 DriverStationLCD.Line.kUser2,
																							 DriverStationLCD.Line.kUser3,
																							 DriverStationLCD.Line.kUser4,
																							 DriverStationLCD.Line.kUser5,
																							 DriverStationLCD.Line.kUser6};
	
	// Vital functionality objects
	private final RobotDrive drivetrain;
  private final Jaguar lift;
  private final Servo cameraServo;
	private final Compressor compressor;
  private final DoubleSolenoid leftSolenoid, rightSolenoid;
	private final Joystick leftStick, rightStick;
	private final DriverStationLCD lcd;
  private final Ultrasonic leftSensor, rightSensor;
	
  public Robot()
	{
		//Initializes functionality variables
	  lastAngleTime = 0;
	  lastReverseTime = 0;
		lastSSTime = 0;
		lastSwitchTime = 0;
	  isCompressorOn = false;
	  isInverted = false;
		isSensitiveAtSlowSpeeds = false;
		lcdLine = 0;

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
    compressor = new Compressor(13, 1);
		compressor.setRelayValue(Relay.Value.kForward);
		compressor.start();

		//Sets up ultrasonic sensors
    leftSensor = new Ultrasonic(PING_CHANNELS[0], ECHO_CHANNELS[0]);
		leftSensor.setEnabled(true);
		leftSensor.setAutomaticMode(true);
    rightSensor = new Ultrasonic(PING_CHANNELS[1], ECHO_CHANNELS[1]);
		rightSensor.setEnabled(true);
		rightSensor.setAutomaticMode(true);
		
		//Sets up Driver Station LCD (User Messages section)
		lcd = DriverStationLCD.getInstance();
		lcdLines = new String[6];
		for (int i = 0; i < 6; i++)
			lcdLines[i] = "";
		//Welcomes Neil, obviously the most important line of code here.
		writeToLCD("Welcome, Neil! C:");
	}

	public void autonomous() 
	{
	  /* TODO: Implement autonomous. */
		System.out.println("Autonomous mode enabled.");
	}

	public void operatorControl() 
	{
	  System.out.println("Teleoperation enabled");
		drivetrain.setSafetyEnabled(false);
		lift.setSafetyEnabled(false);
		
		//Initial solenoid maintenance
		leftSolenoid.set(DoubleSolenoid.Value.kOff);
		rightSolenoid.set(DoubleSolenoid.Value.kOff);
	
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
			if(leftStick.getRawButton(CAMERA_TOGGLE) && (System.currentTimeMillis() - lastAngleTime) >= 250) 
				toggleCameraAngle();
			//Motor inversion control
		  if(rightStick.getRawButton(11) && (System.currentTimeMillis() - lastReverseTime) >= 250)
			  invertMotors();
			//Acceleration curve toggle (linear/exponential)
			if(rightStick.getRawButton(SPEED_SENSITIVITY_TOGGLE) && (System.currentTimeMillis() - lastSSTime) >= 250)
				toggleSpeedSensitivity();
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
		System.out.println("Robot is disabled");
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
		if (isInverted)
			writeToLCD("Motors are inverted. Shooter is front.");
		else
			writeToLCD("Motors are normal. Pick-up is front.");
	}
	
	/**
	 * Toggles the angle of the axis camera. The camera can either be up or down.
	 */
	private void toggleCameraAngle()
	{
		if(isCameraUp)
		{
			cameraServo.set(cameraAngle[0]);
			writeToLCD("Camera view is the arm.");
		}
		else
		{
			cameraServo.set(cameraAngle[1]);
			writeToLCD("Camera view is the field");
		}
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
		if (isSensitiveAtSlowSpeeds)
		{
			writeToLCD("The acceleration curve is exponential.");
			writeToLCD("The joysticks will not be sensitive at slow speeds.");
		}
		else
		{
			writeToLCD("The acceleration curve is linear.");
			writeToLCD("The joysticks will be sensitive at slow speeds.");
		}
		
	}
	
	/**
	 * Writes a line of text to the User Messages section of the Driver Station.
	 * Lines of text are aligned left and may not be longer that 21 characters.
	 * @param text The text to be written to the User Messages box. This String may not be longer than 21 characters.
	 */
	private void writeToLCD(String text)
	{
		if (lcdLine < 6)
		{
			lcd.println(line[lcdLine], 1, text);
			lcdLines[lcdLine] = text;
			lcdLine++;
			lcd.updateLCD();
		}
		else
		{
			lcd.clear();
			for (int i = 1; i < 6; i++)
			{
				lcd.println(line[i-1], 1, lcdLines[i]);
				lcdLines[i-1] = lcdLines[i];
			}
			lcdLines[5] = text;
			lcd.println(line[5], 1, text);
		}
	}
	
	
	/**
	 * Measures distance using the ultrasonic sensors and returns the 
	 * average of the two sensor readings.
	 * @return The distance value as a double, rounded to 2 decimal places.
	 */
	private double measureDistances()
	{
		//I should really use the FRC Dashboard to implement this
		double leftRange = leftSensor.getRangeInches();
		double rightRange = rightSensor.getRangeInches();
		double range = (leftRange + rightRange)/2;
//		String r = Double.toString(range);
//		String[] separated = r.split(".");
//		String end = separated[0] + ".";
//		if (separated[1].length() > 2)
//		{
//			end += separated[1].charAt(0);
//			end += separated[1].charAt(1);
//		}
//		return Double.parseDouble(end); 
		return range;
	}
}
