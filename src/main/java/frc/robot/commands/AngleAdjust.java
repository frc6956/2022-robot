// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AngleAdjust extends CommandBase {
  Drivetrain drivetrain;
  Vision vision;

  /** Creates a new AngleAdjust. */
  public AngleAdjust(Drivetrain drivetrain, Vision vision) {
    addRequirements(drivetrain, vision);
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.getInAngleRange(vision.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.inAngleRange();
  }
}
