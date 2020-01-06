/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// NOT WORKING -- WHEN IT TRIES TO RUN, IT WON'T GO, THEN DISABLES THE ROBOT



package frc.robot.commands;
//package frc.robot.utilities;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.EncoderPIDSource; 
import frc.robot.utilities.SpeedOutput; 
public class NewPIDDrive extends Command {
  public double initialLeftEncoderPosition; 
  public double initialRightEncoderPosition; 
  public final double tenFootDrive = 1550; 
  public double distanceInFeet; 
  public double distanceInClicks; 
  public double currentVelocity;
 

  public PIDController leftDrivePID, rightDrivePID; 
  public EncoderPIDSource leftDriveSource, rightDriveSource; 
  public SpeedOutput leftSpeedOutput, rightSpeedOutput; 
  private double mTargetClicks; 
  private Timer mTimer = new Timer(); 
  public NewPIDDrive(double distanceInFeet) {
  //public NewPIDDrive(double targetFeet) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.mDrive); 
    this.distanceInFeet = distanceInFeet; 

    distanceInClicks = 182 * distanceInFeet;
    mTargetClicks = distanceInClicks; 

    leftDriveSource = new EncoderPIDSource(Robot.mDrive.leftEncoder, PIDSourceType.kDisplacement); 
    leftSpeedOutput = new SpeedOutput(); 
    leftDrivePID = new PIDController(0.3, 0, 0, leftDriveSource, leftSpeedOutput); 
    rightDriveSource = new EncoderPIDSource(Robot.mDrive.rightEncoder, PIDSourceType.kDisplacement); 
    rightSpeedOutput = new SpeedOutput(); 
    rightDrivePID = new PIDController(0.1, 0, .0016, rightDriveSource, rightSpeedOutput); 
    
  }

  // public PIDController rPIDDrivingController = new PIDController(0.078, 0, 0, rightEncoder, rightLeader);
 // public PIDController lPIDDrivingController = new PIDController(0.078, 0, 0, leftEncoder, leftLeader);
     

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.mDrive.resetDriveEncoders();
    Robot.mDrive.resetEncoders();
   leftDrivePID.setAbsoluteTolerance(40);
  leftDrivePID.setContinuous(false);
  leftDrivePID.setOutputRange(-0.6, 0.6);
  leftDrivePID.setSetpoint(mTargetClicks);
  rightDrivePID.setAbsoluteTolerance(40);
  rightDrivePID.setContinuous(false);
  rightDrivePID.setOutputRange(-0.6, 0.6);
  rightDrivePID.setSetpoint(mTargetClicks);
  //SmartDashboard.putNumber("Target Clicks (L)", leftDrivePID.getSetpoint());
 // SmartDashboard.putNumber("Initial Encoder", initialRightEncoderPosition); 
  leftDrivePID.enable();
  rightDrivePID.enable();
  mTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double velocity = rightSpeedOutput.getSpeed();
   // double rSpeed = -rightSpeedOutput.getSpeed();
   Robot.mDrive.startDriveMotors(velocity);
   //Robot.mDrive.startDriveMotors(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
      if(/*!leftDrivePID.onTarget() || */ !rightDrivePID.onTarget()) mTimer.reset();
      if(mTimer.get() > 1) return true;
      else return false;
  
    }
  

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftDrivePID.disable();
    rightDrivePID.disable();
    Robot.mDrive.stopDriveMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end(); 
  }
}
