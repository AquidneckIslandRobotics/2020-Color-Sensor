/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SpinDown;
import frc.robot.commands.SpinUp; 

/**
 * Add your docs here.
 */
public class Frisbeyeet extends Subsystem {
  TalonSRX forwardShooterLeader = new TalonSRX(RobotMap.forwardShooterLeader); 
  TalonSRX forwardShooterFollower = new TalonSRX(RobotMap.forwardShooterFollower); 
  TalonSRX backShooter = new TalonSRX(RobotMap.backShooter);
  TalonSRX shoulder = new TalonSRX(RobotMap.shoulder); 
public Servo blue1 = new Servo(1);
public Servo blue2 = new Servo(2);
public Servo blue3 = new Servo(3);
public Servo red1 = new Servo(4);
public Servo red2 = new Servo(5);
public Servo red3 = new Servo(6);
public DigitalInput backLimit = new DigitalInput(7);
public DigitalInput upperLimit = new DigitalInput(9);
public DigitalInput lowerLimit = new DigitalInput(8);



  public boolean shooting = true; 
//bruh
  public void Shooter() {
    //shoulder.

    
  }


  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    
    // Set the default command for a subsystem here.
     setDefaultCommand(new SpinDown());
  }

  public void initDriveControllers(){
    forwardShooterFollower.follow(forwardShooterLeader);

  }

public void redServoInside(){
red1.set(0);
red2.set(1);
red3.set(1);}

public void blueServoInside(){
blue1.set(.9);
blue2.set(1);
blue3.set(.95);}

public void redServoOutside(){
red1.set(.4);
red2.set(.6);
red3.set(.5);}

public void blueServoOutside(){
blue1.set(.5);
blue2.set(.6);
blue3.set(.5);}

public void spinUp(){
forwardShooterLeader.set(ControlMode.PercentOutput, -.8);
}
public void spinDown(){
  forwardShooterLeader.set(ControlMode.PercentOutput, 0);
}
public void backShooter(){
  backShooter.set(ControlMode.PercentOutput, 1);
}
public void backShooterOff(){
  backShooter.set(ControlMode.PercentOutput, 0);
}
public void setShoulder(double speed){ 
  shoulder.set(ControlMode.PercentOutput, speed);

}
}






