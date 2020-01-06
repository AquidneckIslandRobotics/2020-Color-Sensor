/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Driving;
import edu.wpi.first.wpilibj.command.Command;


public class DriveDistance extends Command {
  public double initialLeftEncoderPosition; 
  public double initialRightEncoderPosition; 
  public final double tenFootDrive = 1550; 
  public double distanceInFeet; 
  public double distanceInClicks; 
  public double currentVelocity;
  public DriveDistance(double distanceInFeet) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.mDrive); 
    this.distanceInFeet = distanceInFeet; 

    distanceInClicks = 155 * distanceInFeet;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    initialLeftEncoderPosition = Robot.mDrive.getLeftEncoder(); 
    initialRightEncoderPosition = Robot.mDrive.getRightEncoder(); 
    SmartDashboard.putNumber("Initial Encoder", initialRightEncoderPosition); 
    currentVelocity = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (currentVelocity > 0.5){
      currentVelocity = 0.5;
    } else{
      double halfClicks = 0.5*distanceInClicks + initialRightEncoderPosition;
      if (halfClicks > Robot.mDrive.getRightEncoder()){
        //if the current clicks are greater than the clicks desired stop increasing the speed
        currentVelocity = ((Robot.mDrive.getRightEncoder()-initialRightEncoderPosition)/155)*0.09+0.2;
      } else {
        currentVelocity = ((Robot.mDrive.getRightEncoder()-initialRightEncoderPosition)/155)*-0.09+.8; 
      }
    }
    SmartDashboard.putNumber("Velocity",currentVelocity);
    Robot.mDrive.startDriveMotors(currentVelocity); 
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.mDrive.getRightEncoder() > initialRightEncoderPosition + distanceInClicks) {
      SmartDashboard.putNumber("Velocity",0);
      return true; 
    }
    else {
      return false; 
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.mDrive.stopDriveMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
