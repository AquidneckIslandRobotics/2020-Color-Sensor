/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.Shooter;

import frc.robot.subsystems.Frisbeyeet;
import frc.robot.subsystems.Driving;

import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj.util.Color; 
import com.revrobotics.ColorSensorV3; 




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final I2C.Port i2cPort = I2C.Port.kOnboard; 
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); 

  public static Driving mDrive = new Driving(); 
  public static OI m_oi;
  public static Subsystem m_Subsystem;
  public static Frisbeyeet mShoot = new Frisbeyeet(); 
  // AHRS ahrs; 


 // public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  // public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    mDrive.initDriveControllers(); 
  mShoot.initDriveControllers();
   // mDrive.resetDriveEncoders(); this was commented out because it was angry
   // mDrive.resetGyro(); 
   // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  

  public void robotPeriodic() {
    Color detectedColor = m_colorSensor.getColor(); 
   double IR = m_colorSensor.getIR(); 
    SmartDashboard.putNumber("Red", detectedColor.red); 
  SmartDashboard.putNumber("Green", detectedColor.green);  
  SmartDashboard.putNumber("Blue", detectedColor.blue);  
  SmartDashboard.putNumber("IR", IR); 
  SmartDashboard.putString("Color", colorDetector(detectedColor));
  int proximity = m_colorSensor.getProximity(); 
  SmartDashboard.putNumber("Proximity", proximity); 
  }
  public String colorDetector(Color detectedColor){
    if(detectedColor.blue > .35){ 
    return "Blue"; 
    }else if(detectedColor.green > .5 && detectedColor.red < .200){
      return "Green";
    }else if(detectedColor.red > .47 && detectedColor.green > .3){
      return "Red";
    }else if(detectedColor.green > .53 && detectedColor.red > .29){
      return "Yellow";
    }else{
      return "Unkown";
    }
  }
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putBoolean("upperLimit", mShoot.upperLimit.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putData("Red Servo Inside", mShoot);
    SmartDashboard.putData("Limit Switch", mShoot.backLimit);
    SmartDashboard.putBoolean("upperLimit", mShoot.upperLimit.get());
    SmartDashboard.putBoolean("lowerLimit", mShoot.lowerLimit.get());
    SmartDashboard.putNumber("Right Encoder", mDrive.getRightEncoder());
    SmartDashboard.putNumber("Left Encoder", mDrive.getLeftEncoder());
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  
  }
}
