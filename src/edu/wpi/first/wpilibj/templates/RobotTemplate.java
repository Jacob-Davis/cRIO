/*
    Test software, modify as needed.
    Copyright ©2014 Jeff Meli.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class RobotTemplate extends SimpleRobot {

    public final RobotDrive motors;
    public final Joystick joy1, joy2;
    public final Solenoid pneumatic1, pneumatic2;

    public RobotTemplate() 
    {
        motors = new RobotDrive(1, 2, 3, 4); //4 Jaguars connected to PWM ports 1-4
        joy1 = new Joystick(1);
        joy2 = new Joystick(2);
        pneumatic1 = new Solenoid(1);
        pneumatic2 = new Solenoid(2);
        
        
    }
    
    public void autonomous() 
    {
        
    }

    public void operatorControl() 
    {
        System.out.println("Driver operation enabled.");
        motors.setSafetyEnabled(false);
        
        while(this.isOperatorControl() && this.isEnabled())
        {
            //motors.tankDrive(joy1, joy2);
            System.out.println("Set 1");
            pneumatic1.set(true);
            pneumatic2.set(false);
            Timer.delay(1);
            System.out.println("Set 2");
            pneumatic1.set(false);
            pneumatic2.set(true);
            Timer.delay(1);
            System.out.println("Set 3");
            pneumatic1.set(true);
            pneumatic2.set(true);
            Timer.delay(1);
            System.out.println("Set 4");
            pneumatic1.set(false);
            pneumatic2.set(false);
            Timer.delay(1);
        }
        
    }

    public void test() 
    {
    
    }
}
