// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RunWhenDisabledCommand extends RunCommand {
  private boolean runsWhenDisabled = true;
  
  public RunWhenDisabledCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  public RunWhenDisabledCommand(Runnable toRun, boolean runWhenDisabled, Subsystem... requirements) {
    super(toRun, requirements);
    runsWhenDisabled = runWhenDisabled;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }
}
