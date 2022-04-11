// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Shoot extends CommandBase {
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  private double minShootSpeed = 2500;

  /** Creates a new Shoot. */
  public Shoot(Intake intake, Feeder feeder, Shooter shooter) {
    addRequirements(intake, feeder, shooter);
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot();
    if(shooter.getRPM() > minShootSpeed) {
      intake.intake();
      feeder.feed();
    } else {
      intake.stop();
      feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
