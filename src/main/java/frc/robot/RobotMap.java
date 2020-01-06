/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Drive Motors
public static final int leftLeader = 7;
public static final int leftFollower = 8;
public static final int rightLeader = 5;
public static final int rightFollower = 6;

// Loading motors
public static final int topLoadingLeader = 5;
public static final int topLoadingFollower1 = 6;
public static final int topLoadingFollower2 = 7;
public static final int topLoadingFollower3 = 8;
public static final int bottomLoadingLeader = 9;
public static final int bottomLoadingFollower1 = 10;
public static final int bottomLoadingFollower2 = 11;
public static final int bottomLoadingFollower3 = 12;
public static final int encoderRA = 2;
public static final int encoderRB = 3;
public static final int encoderLA = 0;
public static final int encoderLB = 1;


// Firing Motors
public static final int forwardShooterLeader = 3;
public static final int forwardShooterFollower = 4;
public static final int backShooter = 1;
public static final int shoulder = 2;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
