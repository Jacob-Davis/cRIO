/*
    SARCS - Semi-Automatic Robot Control System
    v1.4 zeroquarter'

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
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Robot extends SimpleRobot
{
	// General functionality variables
	private long lastAngleTime;
  private long lastReverseTime;
	private long lastSSTime;
	private long lastSwitchTime;
	private long lastFiredTime;
  private boolean isCompressorOn;
  private boolean isInverted;
	private boolean isCameraUp;
	public final boolean USE_MAX_BOTIX = true;
	
	/**
	 * Array of 2 or more distances obtained from ultrasonic sensors each loop, in inches.
	 */
	protected double[] ultrasonicDistances;
	
  /* Buttons */
  public final int FIRE_BUTTON = 1;
  public final int LIFT_UP_BUTTON = 3; //Right stick only
  public final int LIFT_DOWN_BUTTON = 2; //Right stick only
  public final int CAMERA_TOGGLE = 3;
	
	//Deprecated.
  //public final int COMPRESSOR_BUTTON = 5;
	
	public final int SLOW_BUTTON = 4;
	public final int SLOW_BUTTON_2 = 5;
	public final int AUTO_ALIGN_BUTTON = 2; //Left stick only
	
  /* Joystick USB Ports */
  public final int kLeftJoystick = 1;
  public final int kRightJoystick = 2;
	public final double LIFT_SPEED = 0.40;
	
	// Other constants
	public final int[] PING_CHANNELS = {1, 3};
	public final int[] PONG_CHANNELS = {2, 4};
	public final double[] cameraAngle = {0.7, 0.85};

  /* PWM Channels controlling motors */
  public final int[] kDrivetrain = {1, 2, 3, 4};
  public final int kLift = 5;
  public final int kCameraServo = 6; /* XXX: Remember to put a jumper next to this PWM channel */
  public final RobotDrive.MotorType[] motors = {RobotDrive.MotorType.kFrontLeft,
																								RobotDrive.MotorType.kFrontRight,
																								RobotDrive.MotorType.kRearLeft,
																								RobotDrive.MotorType.kRearRight};
	
	//Speeds when operating automatically
	protected double leftDriveSpeed, rightDriveSpeed;
	protected boolean notAligned;
	
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
	private final AnalogChannel maxBotix;
	
	//Really important automatic constants.
	public final double SHOOTING_DISTANCE = 60; //in inches
	public final double SHOOTING_DISTANCE_TOLERANCE = 0.5; //in inches
	public final double AUTO_ALIGN_MIN_SPEED = 0.15; //from -1 to 1
	public final double SLOW_SPEED = 0.15;
	public final double AUTONOMOUS_MODE_SPEED = 0.5; //from -1 to 1
	
  public Robot()
	{
		
		//Initializes functionality variables
	  lastAngleTime = 0;
	  lastReverseTime = 0;
		lastSSTime = 0;
		lastSwitchTime = 0;
	  isCompressorOn = false;
	  isInverted = false;
		lastFiredTime = 0L;
		
		ultrasonicDistances = new double[2];
		ultrasonicDistances[0] = 0D;
		ultrasonicDistances[1] = 0D;
		
		//Initialized automatic functionality variables
		leftDriveSpeed = 0D;
		rightDriveSpeed = 0D;
		notAligned = true;

		//Sets up driving mechanisms (joysticks and drivetrain)
    leftStick = new Joystick(kLeftJoystick);
    rightStick = new Joystick(kRightJoystick);
    drivetrain = new RobotDrive(kDrivetrain[0],
                                kDrivetrain[1],
                                kDrivetrain[2],
                                kDrivetrain[3]);
	  invertMotors();
	  
		maxBotix = new AnalogChannel(2);
		
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
    leftSensor = new Ultrasonic(PING_CHANNELS[0], PONG_CHANNELS[0]);
		leftSensor.setEnabled(true);
    rightSensor = new Ultrasonic(PING_CHANNELS[1], PONG_CHANNELS[1]);
		rightSensor.setEnabled(true);
		
		//Sets up Driver Station LCD (User Messages section)
		lcd = DriverStationLCD.getInstance();
	}

	public void autonomous() 
	{
		System.out.println("Now in auto mode.");
		notAligned = true;
		storeUltrasonicDistances();
		//long startTime = System.currentTimeMillis();
	
	//	while(isEnabled() && notAligned) {
	//		storeUltrasonicDistances();
	//		writeToLCD();
			/*
			if(System.currentTimeMillis() - startTime < 500) {
				lift.set(LIFT_SPEED);
			} else {
				lift.set(0);
			}
			*/
//			if(USE_MAX_BOTIX) {
//				autoAlignMaxBotix();
//			}else {
//				autoAlign();
//			}
//		}
	
		
		long timeStarted = System.currentTimeMillis();
		while(System.currentTimeMillis() - timeStarted < 5500) {
			drivetrain.setLeftRightMotorOutputs(-0.3, -0.42);
		}
		
		
		leftSolenoid.set(DoubleSolenoid.Value.kForward);
	  rightSolenoid.set(DoubleSolenoid.Value.kForward);
		Timer.delay(1);
		leftSolenoid.set(DoubleSolenoid.Value.kReverse);
		rightSolenoid.set(DoubleSolenoid.Value.kReverse);
		Timer.delay(1);
		leftSolenoid.set(DoubleSolenoid.Value.kOff);
		rightSolenoid.set(DoubleSolenoid.Value.kOff);
		drivetrain.setLeftRightMotorOutputs(0, 0);
				
	}

	public void operatorControl() 
	{
	  System.out.println("Now in driver mode.");
		drivetrain.setSafetyEnabled(false);
		lift.setSafetyEnabled(false);
		
		
		
		//Initial solenoid maintenance
		leftSolenoid.set(DoubleSolenoid.Value.kOff);
		rightSolenoid.set(DoubleSolenoid.Value.kOff);
	
		while(this.isOperatorControl() && this.isEnabled()) 
		{
				
			storeUltrasonicDistances();
			
			//Automatic alignment control
			if(leftStick.getRawButton(AUTO_ALIGN_BUTTON) && notAligned == true) {
				if(USE_MAX_BOTIX) {
					autoAlignMaxBotix();
				} else {
					autoAlign();
				}
			} else if(leftStick.getRawButton(SLOW_BUTTON)				) {
					drivetrain.setLeftRightMotorOutputs(-SLOW_SPEED, -SLOW_SPEED);
			} else if(leftStick.getRawButton(SLOW_BUTTON_2)) {
					drivetrain.setLeftRightMotorOutputs(SLOW_SPEED, SLOW_SPEED);
			} else {
					drivetrain.tankDrive(leftStick, rightStick);
					notAligned = true;
			}
			
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
			
	    
			long timeSinceLastFire = System.currentTimeMillis() - lastFiredTime;
			
			/**
			 * Solenoid control. WARNING: Swapping the wire plugs on the cRIO will break this! Be careful!
			 */
		  if(rightStick.getRawButton(FIRE_BUTTON))
			{
				leftSolenoid.set(DoubleSolenoid.Value.kForward);
	      rightSolenoid.set(DoubleSolenoid.Value.kForward);
			} else if(leftStick.getRawButton(FIRE_BUTTON)) {
				leftSolenoid.set(DoubleSolenoid.Value.kReverse);
				rightSolenoid.set(DoubleSolenoid.Value.kReverse);
			} else {
				leftSolenoid.set(DoubleSolenoid.Value.kOff);
				rightSolenoid.set(DoubleSolenoid.Value.kOff);
			}
			
			
		
			//Camera control (up/down toggle)
			if(leftStick.getRawButton(CAMERA_TOGGLE) && (System.currentTimeMillis() - lastAngleTime) >= 500) 
				toggleCameraAngle();
			//Motor inversion control
		  if(rightStick.getRawButton(11) && (System.currentTimeMillis() - lastReverseTime) >= 500)
			  invertMotors();
			
			//Push real-time information to Driver Station LCD (User Messages section)
			writeToLCD();
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
		System.out.println("Robot is connected and disabled.");
	}
	
	public void autoAlignMaxBotix() {
		
		if(ultrasonicDistances[0] < SHOOTING_DISTANCE) {
			drivetrain.setLeftRightMotorOutputs(AUTO_ALIGN_MIN_SPEED, AUTO_ALIGN_MIN_SPEED);
		}
		else if (ultrasonicDistances[0] > SHOOTING_DISTANCE) {
			drivetrain.setLeftRightMotorOutputs(-0.3, -0.4);
		}
		
		if(Math.abs(ultrasonicDistances[0] - SHOOTING_DISTANCE) < SHOOTING_DISTANCE_TOLERANCE) {
			notAligned = false;
		}
	}
	
	/**
	 * Run repeatedly to align the robot.
	 */
	public void autoAlign() {
		
		double leftDistanceFromPerfect = Math.abs(ultrasonicDistances[0] - SHOOTING_DISTANCE);
		double rightDistanceFromPerfect = Math.abs(ultrasonicDistances[1] - SHOOTING_DISTANCE);
		
		if(ultrasonicDistances[0] > SHOOTING_DISTANCE) {
			leftDriveSpeed = AUTO_ALIGN_MIN_SPEED;
		}
		else if(ultrasonicDistances[0] < SHOOTING_DISTANCE) {
			leftDriveSpeed = -AUTO_ALIGN_MIN_SPEED;
		}
		else {
			System.out.println("There was a problem auto-aligning! (1)");
			leftDriveSpeed = 0;
		}
		
		if(ultrasonicDistances[1] > SHOOTING_DISTANCE) {
			rightDriveSpeed = AUTO_ALIGN_MIN_SPEED;
		}
		else if(ultrasonicDistances[1] < SHOOTING_DISTANCE) {
			rightDriveSpeed = -AUTO_ALIGN_MIN_SPEED;
		}
		else {
			System.out.println("There was a problem auto-aligning! (2)");
			rightDriveSpeed = 0;
		}
		
		if(leftDistanceFromPerfect < SHOOTING_DISTANCE_TOLERANCE) {
			leftDriveSpeed = 0;
		}
		if(rightDistanceFromPerfect < SHOOTING_DISTANCE_TOLERANCE) {
			rightDriveSpeed = 0;
		}
		
		if(leftDriveSpeed == 0D && rightDriveSpeed == 0D) {
			notAligned = false;
		}
		
		drivetrain.setLeftRightMotorOutputs(leftDriveSpeed, rightDriveSpeed);
		
	}
	
	public void storeUltrasonicDistances() {
		if(USE_MAX_BOTIX) {
			ultrasonicDistances[0] = getMaxBotixInches();
			ultrasonicDistances[1] = 0;
		} else {
			ultrasonicDistances[0] = leftSensor.getRangeInches();
			ultrasonicDistances[1] = rightSensor.getRangeInches();
		}
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
	
	public double getMaxBotixInches() {
			int reading = maxBotix.getValue();
			System.out.println("Voltage: " + maxBotix.getVoltage() + "\n Reading: " + reading);
			return  (reading - 0.2916666666) / 1.851190476D;
	}
	/**
	 * Toggles the angle of the axis camera. The camera can either be up or down.
	 */
	private void toggleCameraAngle()
	{
		lastAngleTime = System.currentTimeMillis();
		if(isCameraUp)
			cameraServo.set(cameraAngle[0]);
		else
			cameraServo.set(cameraAngle[1]);
		isCameraUp = !isCameraUp;
	}
	
	/**
	 * Writes a line of text to the User Messages section of the Driver Station.
	 * Lines of text are aligned left and may not be longer that 21 characters.
	 * @param text The text to be written to the User Messages box. This String may not be longer than 21 characters.
	 */
	private void writeToLCD()
	{
		lcd.println(Line.kUser1, 1, "Hi Neil! C: SARCS 1.3");
		lcd.println(Line.kUser2, 1, (isInverted) ? "M:Inverted F:Shooter" 
																						 : "M:Normal   F:Pick-up");
		lcd.println(Line.kUser3, 1, (isCameraUp) ? "Camera:Up View:Field"
																						 : "Camera:Dwn View:Arm");
		double left = Math.floor(ultrasonicDistances[0] * 1000) / 1000D;
		double right = Math.floor(ultrasonicDistances[1] * 1000) / 1000D;
		double average = Math.floor(((left + right) / 2D) * 1000) / 1000D;
		lcd.println(Line.kUser4, 1, "LS: " + left + "   RS: " + right);
		lcd.println(Line.kUser5, 1, "Average: " + average);
		lcd.println(Line.kUser6, 1, "http://team1672.com");
						
		lcd.updateLCD();
	}	
}
