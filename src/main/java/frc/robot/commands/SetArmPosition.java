// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;


public class SetArmPosition extends CommandBase {
  
  Climber climber;
  double position;
  /** Creates a new SetArmPosition. */
  public SetArmPosition(final Climber climber, final double newPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(climber);
    position = newPosition;
    this.climber = climber;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getAveragePosition() > position + 1) {
      climber.climbMain(0.3);
    } else if (climber.getAveragePosition() < position - 1){
      climber.climbMain(-0.3);
    } else {
      climber.climbMain(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climbMain(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(position >= 0) {
      if(climber.getAveragePosition() >= position) {
        return true;
      } else {
        return false;
      }
    } else {
      if(climber.getAveragePosition() <= position) {
        return true;
      } else {
        return false;
      }
    }
  }
}
