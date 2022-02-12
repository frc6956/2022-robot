
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final int LeftDriverPort = 0;
  public static final int RightDriverPort = 0;
  // Intake Constants
  public static final int IntakeID = 5;
  public static final double IntakeSpeed = 1.0; 
  // Climber Constants
  public static final int ClimberSideRID = 0; 
  public static final int ClimberSideLID = 0; 
  public static final int ClimberMain1ID = 0; 
  public static final int ClimberMain2ID = 0; 

  public static final double ClimberArmsSpeed = 1.0;
  public static final double ClimberMainSpeed = 1.0;
  // Shooter Constants
  public static final int ShooterMotorLeftID = 0;
  public static final int ShooterMotorRightID = 0;
  public static final double ShooterMotorLeftSpeed = 1.0;
  public static final double ShooterMotorRightSpeed = 1.0;
  // Drivetrain Constants
  public static final int DrivetrainR1ID = 0;
  public static final int DrivetrainL1ID = 0;
  public static final int DrivetrainR2ID = 0;
  public static final int DrivetrainL2ID = 0;
  public static final double DrivetrainR1Speed = 0;
  public static final double DrivetrainL1Speed = 0;
  public static final double DrivetrainR2Speed = 0;
  public static final double DrivetrainL2Speed = 0;
  // Feeder Constants
  public static final int FeederMotorID = 0;
  public static final double FeederMotorSpeed = 1.0; 
  // Input Button Constants
  public static final int IntakeButton = 0;
  public static final int FeederButton = 0;
  public static final int ShooterButton = 0;
  public static final int ClimberMainButton = 0;
  public static final int ClimberArmsButton = 0;
}