/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.              skrt                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Driving;
import frc.robot.Robot;


public class PIDDrive extends Command {
  public PIDDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.mDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double distanceInClicks = 155 * 10;
    Robot.mDrive.lPIDDrivingController.setSetpoint(distanceInClicks);
    Robot.mDrive.rPIDDrivingController.setSetpoint(distanceInClicks*-1);
    Robot.mDrive.lPIDDrivingController.enable();
    Robot.mDrive.rPIDDrivingController.enable();
    
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   // Robot.mDrive.lPIDDrivingController
  // return !(Robot.mDrive.lPIDDrivingController.isEnabled());
  /*if(Robot.mDrive.rPIDDrivingController.onTarget()) {
     return true; 
   }
   else return false; 
   //return false;
*/
   return false; 
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
