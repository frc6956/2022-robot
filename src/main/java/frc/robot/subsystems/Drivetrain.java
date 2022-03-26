// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase { 
private CANSparkMax drivetrainMotorR1;
private CANSparkMax drivetrainMotorR2;
private CANSparkMax drivetrainMotorL1;
private CANSparkMax drivetrainMotorL2;
private DifferentialDrive mainRobotDrive;
private RelativeEncoder drivetrainEncoderR1;
//private SlewRateLimiter filterLeft = new SlewRateLimiter(1.4);
//private SlewRateLimiter filterRight = new SlewRateLimiter(1.4);

  /** Creates a new Drivetrain. */
  public Drivetrain() { 
    drivetrainMotorR1 = new CANSparkMax(Constants.DrivetrainR1ID, MotorType.kBrushless);
    drivetrainMotorR2 = new CANSparkMax(Constants.DrivetrainR2ID, MotorType.kBrushless);
    drivetrainMotorL1 = new CANSparkMax(Constants.DrivetrainL1ID, MotorType.kBrushless);
    drivetrainMotorL2 = new CANSparkMax(Constants.DrivetrainL2ID, MotorType.kBrushless);
    mainRobotDrive = new DifferentialDrive(drivetrainMotorL1, drivetrainMotorR1);
    drivetrainEncoderR1 = drivetrainMotorR1.getEncoder();
    
    drivetrainMotorR1.setInverted(true);
    drivetrainMotorR2.follow(drivetrainMotorR1);
    
    drivetrainMotorL1.setInverted(false);
    drivetrainMotorL2.follow(drivetrainMotorL1);

    resetPosition();
  }
  double position;
  public void tankDrive(double leftSpeed, double rightSpeed) {

    mainRobotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void getInRange(double distance){
    
    if (Constants.autpMinumumRange>distance) {
      tankDrive(-0.3, -0.3);
    } else if (Constants.autoMaximumRange<distance){
      tankDrive(0.3, 0.3);
    } else {
      tankDrive(0, 0);
    }
  }


  public void getInAngleRange(double x){
    if ( x > 1) {
      tankDrive(0.4, -0.4);
    } else if ( x < -3){
      tankDrive(-0.4, 0.4);
    } else {
      tankDrive(0, 0);
    }

  }

  public double getPosition(){
    position = ((6*Math.PI)*drivetrainEncoderR1.getPosition())/10.7; // getPosition returns number of revolutions of the motor
    //wheel diameter is 6 in, motor gear ratio is 10.7
    return position;
  }

  public void resetPosition() {
    drivetrainEncoderR1.setPosition(0);
    
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}