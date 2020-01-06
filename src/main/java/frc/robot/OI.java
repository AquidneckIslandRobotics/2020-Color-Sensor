/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public double getSpeed(){
    return stick.getY(Hand.kLeft); 
  }
  public double getRotation(){
    return stick.getX(Hand.kRight); 

  }
  public boolean getQuickTurn(){
    return button.get(); 
    
  }
  
  XboxController stick = new XboxController(1);
  Button button = new JoystickButton(stick, 2); 
  Button button2 = new JoystickButton(stick, 4);
  Button button3 = new JoystickButton(stick, 3);
  Button button4 = new JoystickButton(stick, 5);
  Button button5 = new JoystickButton(stick, 6);
  Button button7 = new JoystickButton(stick, 7); 
  Button button8 = new JoystickButton(stick, 8); 
  Button button9 = new JoystickButton(stick, 9);
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());
  public OI(){
button2.whenPressed(new FrisYeet());
button3.whenPressed(new BackShooter());
button4.whileHeld(new ShoulderLower());
//nice
button5.whileHeld(new ShoulderRaise());
button7.whenPressed(new DriveDistance(10));
button8.whenPressed(new DriveDistance(5));
//button9.whenPressed(new PIDDrive());
button9.whenPressed(new NewPIDDrive(5)); 
  }


  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
