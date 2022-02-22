// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;


  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor = new CANSparkMax(Constants.FeederMotorID, MotorType.kBrushless);
    feederMotor.restoreFactoryDefaults();
    feederMotor.setInverted(true);
  }
  
  public void feed(double speed) {
    feederMotor.set(speed);
  }
  
  public void stop() {
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
