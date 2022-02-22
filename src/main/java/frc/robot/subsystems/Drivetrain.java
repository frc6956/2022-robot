// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase { 
private CANSparkMax drivetrainMotorR1;
private CANSparkMax drivetrainMotorR2;
private CANSparkMax drivetrainMotorL1;
private CANSparkMax drivetrainMotorL2;
private DifferentialDrive mainRobotDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() { 
    drivetrainMotorR1 = new CANSparkMax(Constants.DrivetrainR1ID, MotorType.kBrushless);
    drivetrainMotorR2 = new CANSparkMax(Constants.DrivetrainR2ID, MotorType.kBrushless);
    drivetrainMotorL1 = new CANSparkMax(Constants.DrivetrainL1ID, MotorType.kBrushless);
    drivetrainMotorL2 = new CANSparkMax(Constants.DrivetrainL2ID, MotorType.kBrushless);
    mainRobotDrive = new DifferentialDrive(drivetrainMotorL1, drivetrainMotorR1);

    
    drivetrainMotorR1.setInverted(true);
    drivetrainMotorR2.follow(drivetrainMotorR1);
    
    drivetrainMotorL1.setInverted(false);
    drivetrainMotorL2.follow(drivetrainMotorL1);
    
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    mainRobotDrive.tankDrive(leftSpeed, rightSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}