// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArms;

public class SetSideArmPosition extends CommandBase {

  ClimberArms climberArms;
  double position;
  double maxRange = 0.07;
  /** Creates a new SetSideArmPosition. */
  public SetSideArmPosition(final ClimberArms climberArms, double newPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberArms);
    this.climberArms = climberArms;
    position = newPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if (climber.getLeftArmCurrent() < 30 || climber.getRightArmCurrent() < 30){
      climber.climbSide(0.2);
      climber.resetArmPosition();
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberArms.getLeftArmPosition() > position + maxRange){
      climberArms.setLeft(-0.07);
    } else if (climberArms.getLeftArmPosition() < position - maxRange){
      climberArms.setLeft(0.07);
    }
    
    if (climberArms.getRightArmPosition() > position + maxRange){
      climberArms.setRight(-0.07);
    } else if (climberArms.getRightArmPosition() < position - maxRange){
      climberArms.setRight(0.07);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberArms.climbSide(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(position >= 0) {
      if((climberArms.getLeftArmPosition() >= position) && (climberArms.getRightArmPosition() >= position)) {
        return true;
      }else {
        return false;
      }

    }else {
      if (climberArms.getLeftArmPosition() <= position && climberArms.getRightArmPosition() <= position) {
        return true;
      } else {
        return false;
      }
    }
  }
}
