// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotorMain1;
  private CANSparkMax climberMotorMain2;
  
  
  
  /** Creates a new Climber. */
  public Climber() {
    climberMotorMain1 = new CANSparkMax(Constants.ClimberMain1ID, MotorType.kBrushless);
    climberMotorMain2 = new CANSparkMax(Constants.ClimberMain2ID, MotorType.kBrushless);
    climberMotorMain1.restoreFactoryDefaults();
    climberMotorMain2.restoreFactoryDefaults();
    
    climberMotorMain1.setIdleMode(IdleMode.kBrake);
    climberMotorMain2.setIdleMode(IdleMode.kBrake);
  }

  public void climbMain(double speed) {
    climberMotorMain1.set(-speed);
    climberMotorMain2.set(speed);
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

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
