/*
    SARCS (Semi-Automatic Robot Control System)
    v0.2 'coolblock'
    
    This code is to be run on the cRIO.

    Copyright �2014 Jeff Meli, Tommy Bohde.

    SARCS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SARCS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SARCS.  If not, see <http://www.gnu.org/licenses/>.
*/
package com.team1672.FRC2014;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SimpleRobot {

    protected long ticks;
    protected long timeOfFirstTick;
    
    public final int FIRE_BUTTON = 1;
    public final int TOGGLE_DRIVE_MODE_BUTTON = 2;
    public final int COMPRESSOR_BUTTON = 3;
    
    public final RobotDrive motors;
    public final Joystick leftStick, rightStick;
    public final DoubleSolenoid pneumatic1, pneumatic2;
    public final Relay compressor;
    protected boolean tankDrive;
    protected boolean isCompressorOn;

    public Robot() 
    {
        motors = new RobotDrive(1, 2, 3, 4); //4 Jaguars connected to PWM ports 1-4
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        pneumatic1 = new DoubleSolenoid(1, 2);
        pneumatic1.set(DoubleSolenoid.Value.kOff);
        
        pneumatic2 = new DoubleSolenoid(3, 4);
        pneumatic2.set(DoubleSolenoid.Value.kOff);
        
        compressor = new Relay(1);
        compressor.set(Relay.Value.kOff);
        isCompressorOn = false;
        
        tankDrive = true;
        
        ticks = 1;
        timeOfFirstTick = System.currentTimeMillis();
    }
    
    public void autonomous() 
    {
        System.out.println("Autonomous mode has no purpose currently; switch to manual operation.");
    }

    public void operatorControl() 
    {
        System.out.println("Driver operation enabled. Using tank drive mode.");
        motors.setSafetyEnabled(false);
        long lastToggle = 0;
        long lastTimeCompressorToggled = 0;
        
        while(this.isOperatorControl() && this.isEnabled())
        {
            if(tankDrive)
            {
                motors.tankDrive(leftStick, rightStick);
            }
            else
            {
                motors.arcadeDrive(leftStick);
            }
            
            if(leftStick.getRawButton(COMPRESSOR_BUTTON) && System.currentTimeMillis() - lastTimeCompressorToggled > 250L)
            {
                lastTimeCompressorToggled = System.currentTimeMillis();
                if(isCompressorOn)
                    compressor.set(Relay.Value.kOff);
                else
                    compressor.set(Relay.Value.kForward);
                isCompressorOn = !isCompressorOn;
            }

            if(rightStick.getRawButton(TOGGLE_DRIVE_MODE_BUTTON) && System.currentTimeMillis() - lastToggle > 500L)
            {
                tankDrive = !tankDrive;
                lastToggle = System.currentTimeMillis();
            }
            
            //TOUCH ME AND I KILL YOU
            if(rightStick.getRawButton(FIRE_BUTTON))
            {
                pneumatic1.set(DoubleSolenoid.Value.kForward);
                long buttonPressed = System.currentTimeMillis();
                pneumatic2.set(DoubleSolenoid.Value.kForward);
                System.out.println("Time between piston fires: " + (System.currentTimeMillis() - buttonPressed));
            }
            else if(leftStick.getRawButton(FIRE_BUTTON))
            {
                pneumatic1.set(DoubleSolenoid.Value.kReverse);
                pneumatic2.set(DoubleSolenoid.Value.kReverse);
            }
            else
            {
                pneumatic1.set(DoubleSolenoid.Value.kOff);
                pneumatic2.set(DoubleSolenoid.Value.kOff);
            }
            if(ticks % 100 == 0)
            {
                // System.out.println("Current tick: " + ticks);
            }
            ticks++;
        }
    }
      
    public void test() 
    {
        System.out.println("Test mode enabled. \n");
        System.out.println(motors.getDescription() + ", " + motors.toString());
        System.out.println(leftStick.toString());
        System.out.println(rightStick.toString());
        System.out.println(pneumatic1.toString());
    }
      
}
