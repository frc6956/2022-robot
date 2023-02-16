// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotorMain1;
  private CANSparkMax climberMotorMain2;
  private RelativeEncoder climberEncoder1;
  private RelativeEncoder climberEncoder2;
  
  
  
  /** Creates a new Climber. */
  public Climber() {
    climberMotorMain1 = new CANSparkMax(Constants.ClimberMain1ID, MotorType.kBrushless);
    climberMotorMain2 = new CANSparkMax(Constants.ClimberMain2ID, MotorType.kBrushless);
    climberMotorMain1.restoreFactoryDefaults();
    climberMotorMain2.restoreFactoryDefaults();
    climberEncoder1 = climberMotorMain1.getEncoder();
    climberEncoder2 = climberMotorMain2.getEncoder();
    
    climberMotorMain1.setIdleMode(IdleMode.kBrake);
    climberMotorMain2.setIdleMode(IdleMode.kBrake);

    resetPosition();
  }

  public void climbMain(double speed) {
    if ((getAveragePosition() > 150) && (speed < 0)){
      stopMain();
    } else {
      climberMotorMain1.set(-speed);
      climberMotorMain2.set(speed);
    }
  }

  public void climbLeft(double speed){
    climberMotorMain1.set(-speed);
  }

  public void climbRight(double speed){
    climberMotorMain2.set(speed); 
  }

  public void stopMain() {
    climberMotorMain1.set(0);
    climberMotorMain2.set(0);
  }

  public void resetPosition() {
    climberEncoder1.setPosition(0);
    climberEncoder2.setPosition(0);
    
  }

  public void setArmPositionClimber(){
    if (getAveragePosition() > Constants.getInPositionNum + 1) {
      climbMain(0.25);
    } else if (getAveragePosition() < Constants.getInPositionNum - 1){
      climbMain(-0.25);
    } else {
      climbMain(0);
    }
  }

  public void returnArmPositionClimber(){
    if (getAveragePosition() > 1) {
      climbMain(0.25);
    } else if (getAveragePosition() < -1){
      climbMain(0.25);
    } else {
      climbMain(0);
    }
  }

  
  public double getPosition1(){
    double position = (((3/4)*Math.PI)*climberEncoder1.getPosition())/16; // getPosition returns number of revolutions of the motor
    // motor gear ratio is 16
    return position;
  }

  public double getPosition2(){
    double position = -(((3/4)*Math.PI)*climberEncoder2.getPosition())/16; // getPosition returns number of revolutions of the motor
    // motor gear ratio is 16
    return position;
  }

  public double getAveragePosition(){
    double position = (climberEncoder1.getPosition() + -(climberEncoder2.getPosition()))/2;
    return position;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Main Climber Arm Encoder", climberEncoder1.getPosition());
    SmartDashboard.putNumber("Main Climber Arm 2 Encoder", climberEncoder2.getPosition());
    SmartDashboard.putNumber("Average Main Encoder Position", getAveragePosition());
  }
}
