
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Driver Station Ports
  public static final int OperatorPort = 0;
  public static final int LeftDriverPort = 1;
  public static final int RightDriverPort = 2;
  // Intake Constants
  public static final int IntakeID = 5;
  public static final double IntakeSpeed = 0.75; 
  // Climber Constants
  public static final int ClimberSideRID = 14; 
  public static final int ClimberSideLID = 11; 
  public static final int ClimberMain1ID = 12; //left main
  public static final int ClimberMain2ID = 13; //right main

  public static final double ClimberArmsSpeed = 0.25;
  public static final double ClimberArmsReverseSpeed = -0.25;
  public static final double ClimberMainSpeed = 0.25;
  // Shooter Constants
  public static final int ShooterMotorLeftID = 4;
  public static final int ShooterMotorRightID = 2;
  public static final int AuxShooterMoterLeftID = 15;
  public static final int AuxShooterMoterRightID = 16;
  //public static final double ShooterMotorLeftSpeed = 0.45; //50
  public static final double ShooterMotorRightSpeed = 0.40; //50 // auton 50 75 regular 40
  public static final double AuxShooterMotorRightSpeed = 0.60; //75 regular 60
  public static final double AutoShooterMotorRightSpeed = 0.40;
  public static final double AutoAuxShooterMotorRightSpeed = 0.60;
  // Drivetrain Constants
  public static final int DrivetrainR1ID = 8;
  public static final int DrivetrainL1ID = 6;
  public static final int DrivetrainR2ID = 9;
  public static final int DrivetrainL2ID = 7;
  public static final double DrivetrainR1Speed = 0; //not used
  public static final double DrivetrainL1Speed = 0; //not used
  public static final double DrivetrainR2Speed = 0; //not used
  public static final double DrivetrainL2Speed = 0; //not used
  // Feeder Constants
  public static final int FeederMotorID = 3;
  public static final double FeederMotorSpeed = 1; 
  public static final double FeederMotorReverseSpeed = -0.5; 
  // Input Button Constants
  public static final int IntakeButton = 11;
  public static final int IntakeReverseButton = 12;
  public static final int FeederButton = 1; //trigger
  public static final int FeederReverseButton = 3;
  public static final int ShooterButton = 2;
  public static final int ClimberMainButtonDown = 4;
  public static final int ClimberMainButtonUp = 4;
  public static final int ClimberArmsButtonForward = 9;
  public static final int ClimberArmsButtonBack = 10;
  public static final int VisionButton = 5;
  public static final int InRangeButton = 1;
  public static final int LEDCelebrateButton = 8;
  public static final int ClimberLeftMainButton = 6;
  public static final int ClimberRightMainButton = 5;
  public static final int ShooterLowButton = 7;
  public static final int ApriltagButton = 11;
  public static final int VisionTapeButton = 12;
  public static final int ClimberMainPosition = 3;
  public static final int ClimberMainPositionHigh = 4;
  public static final int ClimberMainReturnPosition = 2;
  // Range Constants
  /*



  */
  //ideal is 110
  public static final int minumumRange = 36; // 103
  public static final int maximumRange = 40; // 120
  //ideal is 110
  public static final int autoMinumumRange = 36; // 107 //98
  public static final int autoMaximumRange = 40; // 115 // 102

  public static final int getInPositionNum = 55;


  //PID Constants for Path Planning
  public static final double ksVolts = (0.10184 + 0.14712 + 0.15174 + 0.20941 + 0.20858)/5;
  public static final double kvVoltSecondsPerMeter = (1.289 + 1.2671 + 1.2599 + 1.3173 + 1.3129)/5;
  public static final double kaVoltSecondsSquaredPerMeter = (0.68114 + 0.66353 + 0.65203 + 0.20653 + 0.23054)/5;
  public static final double kTrackwidthMeters = 0.56515; 
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kPDriveVel = (1.3185 + 1.5731 + 1.564 + 1.4618 + 1.484)/5;


  //PID coefficients
  //public static final double kD=1;
  //public static final double kI=1;
  //public static final double kP=1;

  // new tested min = __in from vision target (parallel to ground)
  // new tested max = 80in from vision target (parallel to ground)
  // not min but still good = 72in from vision target (parallel to ground)

}