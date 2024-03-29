// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArms;



public class SetArmPositionHigh extends SetArmPosition {

  private Climber climber;
  /** Creates a new SetArmPositionHigh. */
  public SetArmPositionHigh(final Climber climber) {
    super(climber, 120); 
    
    addRequirements(climber);
    
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
