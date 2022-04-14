// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberArms;

public class CalibrateClimberArms extends CommandBase {
  ClimberArms arms;
  boolean leftDone;
  boolean rightDone;
  int leftCount;
  int rightCount;

  /** Creates a new CalibrateClimberArms. */
  public CalibrateClimberArms(ClimberArms arms) {
    addRequirements(arms);
    this.arms = arms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftDone = false;
    rightDone = false;
    leftCount = 0;
    rightCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!leftDone) {
      arms.setLeft(Constants.ClimberArmsReverseSpeed);
      if(arms.getLeftArmCurrent() > 30.0) {
        leftCount++;
        if(leftCount > 10) {
          arms.setLeftArmAngle(-45);
          leftDone = true;
        }
      }
    } else {
      arms.setLeft(0);
    }

    if(!rightDone) {
      arms.setRight(Constants.ClimberArmsReverseSpeed);
      if(arms.getRightArmCurrent() > 30.0) {
        rightCount++;
        if(rightCount > 10) {
          arms.setRightArmAngle(-45);
          rightDone = true;
        }
      }
    } else {
      arms.setRight(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.setArmsVertical();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (leftDone && rightDone);
  }
}
