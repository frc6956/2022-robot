// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;


public class Drivetrain extends SubsystemBase { 
private CANSparkMax drivetrainMotorR1;
private CANSparkMax drivetrainMotorR2;
private CANSparkMax drivetrainMotorL1;
private CANSparkMax drivetrainMotorL2;
private DifferentialDrive mainRobotDrive;
private RelativeEncoder drivetrainEncoderR1;
private RelativeEncoder drivetrainEncoderL1;
private RelativeEncoder drivetrainEncoderR2;
private RelativeEncoder drivetrainEncoderL2;
private SparkMaxPIDController drivetrainPIDControllerL;
private SparkMaxPIDController drivetrainPIDControllerR;
private WPI_PigeonIMU m_gyro;

private Vision vision;
private final Field2d m_field;

private final DifferentialDriveOdometry m_odometry;

double kP=0.5;
double kI=0;
double kD=0;
double kIz=0;
double kFF=0;
double kMaxOutput=0;
double kMinOutput=0;
double maxRPM=0;

  /** Creates a new Drivetrain. */
  public Drivetrain(WPI_PigeonIMU m_gyro, final Vision vision) { 

    m_field = new Field2d();

    this.vision = vision;

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_field);
    

    drivetrainMotorR1 = new CANSparkMax(Constants.DrivetrainR1ID, MotorType.kBrushless);
    drivetrainMotorR2 = new CANSparkMax(Constants.DrivetrainR2ID, MotorType.kBrushless);
    drivetrainMotorL1 = new CANSparkMax(Constants.DrivetrainL1ID, MotorType.kBrushless);
    drivetrainMotorL2 = new CANSparkMax(Constants.DrivetrainL2ID, MotorType.kBrushless);

    drivetrainMotorL1.restoreFactoryDefaults();
    drivetrainMotorL2.restoreFactoryDefaults();
    drivetrainMotorR1.restoreFactoryDefaults();
    drivetrainMotorR2.restoreFactoryDefaults();

    mainRobotDrive = new DifferentialDrive(drivetrainMotorL1, drivetrainMotorR1);

    drivetrainEncoderR1 = drivetrainMotorR1.getEncoder();
    drivetrainEncoderL1 = drivetrainMotorL1.getEncoder();
    drivetrainEncoderR2 = drivetrainMotorR2.getEncoder();
    drivetrainEncoderL2 = drivetrainMotorL2.getEncoder();
    
    drivetrainMotorL1.enableVoltageCompensation(12); //Lower end of voltage
    drivetrainMotorL2.enableVoltageCompensation(12); //Lower end of voltage
    drivetrainMotorR1.enableVoltageCompensation(12); //Lower end of voltage
    drivetrainMotorR2.enableVoltageCompensation(12); //Lower end of voltage

    


    this.m_gyro = m_gyro;
    double encoderVelocityConversionFactor = ((6*Math.PI*0.0254)/60)/10.71;
    double encoderPositionConversionFactor = ((6*Math.PI*0.0254))/10.71;
    drivetrainEncoderL1.setPositionConversionFactor(encoderPositionConversionFactor);
    drivetrainEncoderR1.setPositionConversionFactor(encoderPositionConversionFactor);
    drivetrainEncoderL1.setVelocityConversionFactor(encoderVelocityConversionFactor);
    drivetrainEncoderR1.setVelocityConversionFactor(encoderVelocityConversionFactor);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), drivetrainEncoderL1.getPosition(), drivetrainEncoderR1.getPosition());
    
    drivetrainMotorR1.setInverted(true);
    drivetrainMotorR2.follow(drivetrainMotorR1);
    
    drivetrainMotorL1.setInverted(false);
    drivetrainMotorL2.follow(drivetrainMotorL1);

    SparkMaxPIDController drivetrainPIDControllerR= drivetrainMotorR1.getPIDController();
    SparkMaxPIDController drivetrainPIDControllerL= drivetrainMotorL1.getPIDController();

    drivetrainPIDControllerL.setP(kP);
    drivetrainPIDControllerL.setI(kI);
    drivetrainPIDControllerL.setD(kD);
    drivetrainPIDControllerL.setIZone(kIz);
    drivetrainPIDControllerL.setFF(kFF);
    drivetrainPIDControllerL.setOutputRange(kMinOutput, kMaxOutput);

    drivetrainPIDControllerR.setP(kP);
    drivetrainPIDControllerR.setI(kI);
    drivetrainPIDControllerR.setD(kD);
    drivetrainPIDControllerR.setIZone(kIz);
    drivetrainPIDControllerR.setFF(kFF);
    drivetrainPIDControllerR.setOutputRange(kMinOutput, kMaxOutput);

    

    resetPosition();
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
      drivetrainPIDControllerR.setP(p);
      drivetrainPIDControllerL.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      drivetrainPIDControllerR.setI(i);
      drivetrainPIDControllerL.setI(i);

      kI = i;
    }
    if ((d != kD)) {
      drivetrainPIDControllerR.setD(d);
      drivetrainPIDControllerL.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      drivetrainPIDControllerR.setIZone(iz);
      drivetrainPIDControllerL.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      drivetrainPIDControllerR.setFF(ff);
      drivetrainPIDControllerL.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      drivetrainPIDControllerR.setOutputRange(min, max);
      drivetrainPIDControllerL.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  double position;
  public void tankDrive(double leftSpeed, double rightSpeed) {
    mainRobotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void getInRange(double distance){
    if (Constants.autoMinumumRange>distance) {
      tankDrive(0.25, 0.25);
    } else if (Constants.autoMaximumRange<distance){
      tankDrive(-0.25, -0.25);
    } else {
      tankDrive(0, 0);
    }
  }


  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    drivetrainMotorL1.setVoltage(leftVolts);
    drivetrainMotorR1.setVoltage(rightVolts);
    mainRobotDrive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(drivetrainEncoderL1.getVelocity(), drivetrainEncoderR1.getVelocity());
  }


  public void getInAngleRange(double x){
    double offset = 2.0;
    double multiplier = 0.1;
    double output = Math.min(0.4, Math.abs((x + offset) * multiplier));

    output = Math.copySign(output, x + offset);

    tankDrive(output, -output);
  }

  public void getInAngleRange2(double x){
    if ( x > 0) {
      tankDrive(0.4, -0.4);
    } else if ( x < -4){
      tankDrive(-0.4, 0.4);
    } else {
      tankDrive(0, 0);
    }

  }

  public void getInAllRange(double distance, double xAngle){
    if (xAngle > 1.5) {
      tankDrive(-0.4, -0.4);
    } else if (xAngle < -1.5) {
      tankDrive(0.4, 0.4);
    } else if (Constants.autoMinumumRange>distance) {
      tankDrive(0.4, -0.4);
    } else if (Constants.autoMaximumRange<distance) {
      tankDrive(-0.4, 0.4);
    } else {
      tankDrive(0, 0);
    }
  }


  public double getDistance(RelativeEncoder encoder){
    position = encoder.getPosition(); // getPosition returns number of revolutions of the motor
    //wheel diameter is 6 in, motor gear ratio is 10.7
    return position;
  }

  public double getPosition(){
    return drivetrainEncoderL1.getPosition();
  }

  public void resetPosition() {
    drivetrainEncoderR1.setPosition(0);
    drivetrainEncoderL1.setPosition(0);
  }


  public void resetOdometry(Pose2d pose){
    resetPosition();
    zeroHeading();
    m_odometry.resetPosition(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1), pose);
  }

  public void resetOdometry(){
    resetOdometry(new Pose2d());
  }

  public void resetOdometry(double x, double y, double radians){
    resetOdometry(new Pose2d(x, y, new Rotation2d(radians)));
  }

  public double getAverageEncoderDistance(){
    return (getDistance(drivetrainEncoderL1) + getDistance(drivetrainEncoderR1)) / 2;
  }

  public RelativeEncoder getLefRelativeEncoder(){
    return drivetrainEncoderL1;
  }

  public RelativeEncoder getRighRelativeEncoder(){
    return drivetrainEncoderR1;
  }

  public void setMaxOutput(double maxOutput){
    mainRobotDrive.setMaxOutput(maxOutput);
  }
  
  public void zeroHeading(){
    m_gyro.reset();
  }

  public double getHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -m_gyro.getAngle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    m_odometry.update(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1));

  
    if (vision.hasTarget() && (!DriverStation.isAutonomous() || !DriverStation.isEnabled()) && vision.getDistance() < 140){
      Pose2d pose = vision.getLimelightPose();
      if (pose != null) {
        resetOdometry(pose);
      }
    }

    m_odometry.update(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Drivetrain Left Encoder Velocity", drivetrainEncoderL1.getVelocity());
    SmartDashboard.putNumber("Drivetrain Right Encoder Velocity", drivetrainEncoderR1.getVelocity());
    SmartDashboard.putNumber("Drivetrain Left Encoder Distance", getDistance(drivetrainEncoderL1));
    SmartDashboard.putNumber("Drivetrain Right Encoder Distance", getDistance(drivetrainEncoderR1));
  }
}