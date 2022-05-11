// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax ShooterMotorRight; // Creates new motor
  private CANSparkMax ShooterMotorLeft;
  private CANSparkMax AuxShooterMotorRight;
  private CANSparkMax AuxShooterMotorLeft;
  private RelativeEncoder shooterMainEncoder; // creates encoder for one of the motors
  private RelativeEncoder auxRelativeEncoder; // creates encoder for one of the motors

  private SparkMaxPIDController shooterPIDController;

  // PID coefficients
  double kP = 6e-5;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000015;
  double kMaxOutput = 1;
  double kMinOutput = -1;
  double maxRPM = 6000;

  public Shooter() { // Defines both the Ids and the type of motors that were created above
    ShooterMotorLeft = new CANSparkMax(Constants.ShooterMotorLeftID, MotorType.kBrushless);
    ShooterMotorRight = new CANSparkMax(Constants.ShooterMotorRightID, MotorType.kBrushless);
    AuxShooterMotorLeft = new CANSparkMax(Constants.AuxShooterMoterLeftID, MotorType.kBrushless);
    AuxShooterMotorRight = new CANSparkMax(Constants.AuxShooterMoterRightID, MotorType.kBrushless);

    ShooterMotorLeft.restoreFactoryDefaults();
    ShooterMotorRight.restoreFactoryDefaults();
    AuxShooterMotorLeft.restoreFactoryDefaults();
    AuxShooterMotorRight.restoreFactoryDefaults();

     ShooterMotorLeft.enableVoltageCompensation(12);
     ShooterMotorRight.enableVoltageCompensation(12);
     AuxShooterMotorLeft.enableVoltageCompensation(12);
     AuxShooterMotorRight.enableVoltageCompensation(12);

     ShooterMotorRight.setInverted(true);
     ShooterMotorLeft.follow(ShooterMotorRight, true);
     //AuxShooterMotorRight.follow(ShooterMotorRight, false);
     AuxShooterMotorRight.setInverted(false);
     AuxShooterMotorLeft.follow(AuxShooterMotorRight, true);

    shooterMainEncoder = ShooterMotorRight.getEncoder();
    auxRelativeEncoder = AuxShooterMotorRight.getEncoder();
    shooterPIDController = ShooterMotorRight.getPIDController();

    // set PID coefficients
    shooterPIDController.setP(kP);
    shooterPIDController.setI(kI);
    shooterPIDController.setD(kD);
    shooterPIDController.setIZone(kIz);
    shooterPIDController.setFF(kFF);
    shooterPIDController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  public void updatePID() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      shooterPIDController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      shooterPIDController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      shooterPIDController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      shooterPIDController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      shooterPIDController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      shooterPIDController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  public void shoot() { // Sets speed of motors to the speed constants when called upon
    ShooterMotorRight.set(Constants.ShooterMotorRightSpeed);
    AuxShooterMotorRight.set(Constants.AuxShooterMotorRightSpeed);
    getRPM();
  }

  public void shootLow() { // Sets speed of motors to the speed constants when called upon
    ShooterMotorRight.set(0.20);
    AuxShooterMotorRight.set(0.20);
    getRPM();
  }

  public void autoShoot() { // Sets speed of motors to the speed constants when called upon
    ShooterMotorRight.set(Constants.AutoShooterMotorRightSpeed);
    AuxShooterMotorRight.set(Constants.AutoAuxShooterMotorRightSpeed);
    getRPM();
  }

  public void stop() { // Stops the motors when calle upon
    ShooterMotorLeft.set(0);
    ShooterMotorRight.set(0);
    AuxShooterMotorRight.set(0);
    AuxShooterMotorLeft.set(0);
  }

  public double getRPM() { // gets the rpm of one of the shooter motors from the sparkMax encoder
    return shooterMainEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterRPM", shooterMainEncoder.getVelocity());
    SmartDashboard.putNumber("AuxShooterRPM", auxRelativeEncoder.getVelocity());
  }
}
