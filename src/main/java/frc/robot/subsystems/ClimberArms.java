// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class ClimberArms extends SubsystemBase {
  private CANSparkMax climberMotorSideR;
  private CANSparkMax climberMotorSideL;
  
  
  /** Creates a new Climber. */
  public ClimberArms() {
    climberMotorSideR = new CANSparkMax(Constants.ClimberSideRID, MotorType.kBrushless);
    climberMotorSideL = new CANSparkMax(Constants.ClimberSideLID, MotorType.kBrushless);

    climberMotorSideR.restoreFactoryDefaults();
    climberMotorSideL.restoreFactoryDefaults();

    climberMotorSideR.setIdleMode(IdleMode.kBrake);
    climberMotorSideL.setIdleMode(IdleMode.kBrake);
  }

  public void climbSide(double speed) {
    climberMotorSideR.set(speed*.5);
    climberMotorSideL.set(-speed*.5);
  }

  public void stopSide() {
    climberMotorSideR.set(0);
    climberMotorSideL.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
