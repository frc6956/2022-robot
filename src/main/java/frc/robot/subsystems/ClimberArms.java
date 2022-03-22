// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class ClimberArms extends SubsystemBase {
  private CANSparkMax climberMotorSideR;
  private CANSparkMax climberMotorSideL;
  private RelativeEncoder climberRightEncoder;
  private RelativeEncoder climberLeftEncoder;
  
  
  double leftErrorPosition = 0;
  double lastLeftArmPosition = 0;

  double rightErrorPosition = 0;
  double lastRightArmPosition = 0;

  /** Creates a new Climber. */
  public ClimberArms() {
    climberMotorSideR = new CANSparkMax(Constants.ClimberSideRID, MotorType.kBrushless);
    climberMotorSideL = new CANSparkMax(Constants.ClimberSideLID, MotorType.kBrushless);

    climberMotorSideR.restoreFactoryDefaults();
    climberMotorSideL.restoreFactoryDefaults();

    climberMotorSideR.setIdleMode(IdleMode.kBrake);
    climberMotorSideL.setIdleMode(IdleMode.kBrake);

    climberLeftEncoder = climberMotorSideL.getEncoder();
    climberRightEncoder = climberMotorSideR.getEncoder();

    lastLeftArmPosition = getLeftArmPosition();
    lastRightArmPosition = getRightArmPosition();
  }




  public void climbSide(double speed) {
    climberMotorSideR.set(speed*.5);
    climberMotorSideL.set(-speed*.5);
    lastRightArmPosition = getRightArmPosition();
    lastLeftArmPosition = getLeftArmPosition();
  }

  public void climbSideAndHold(double speed) {
    double leftArmPosition = getLeftArmPosition();

    if (speed < -0.1 && speed > 0.1){
      climberMotorSideL.set(speed*.5);
      lastLeftArmPosition = leftArmPosition;
    } else {
      leftErrorPosition = leftArmPosition - lastLeftArmPosition;
      climberMotorSideL.set(leftErrorPosition);
    }

    double rightArmPosition = getRightArmPosition();

    if (speed < -0.1 && speed > 0.1){
      climberMotorSideR.set(speed*.5);
      lastRightArmPosition = rightArmPosition;
    } else {
      rightErrorPosition = rightArmPosition - lastRightArmPosition;
      climberMotorSideR.set(rightErrorPosition);
    }
  }

  public void stopSide() {
    climberMotorSideR.set(0);
    climberMotorSideL.set(0);
  }

  public void hold() {
    double leftArmPosition = getLeftArmPosition();
    leftErrorPosition = leftArmPosition - lastLeftArmPosition;
    climberMotorSideL.set(-leftErrorPosition*2);

    double rightArmPosition = getRightArmPosition();
    rightErrorPosition = rightArmPosition - lastRightArmPosition;
    climberMotorSideR.set(-rightErrorPosition*2);
  }

  public double getLeftArmPosition(){
    double leftPosition = climberLeftEncoder.getPosition() / 64;
    return leftPosition;
  }

  public double getRightArmPosition(){
    double rightPosition = climberRightEncoder.getPosition() / 64;
    return rightPosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
