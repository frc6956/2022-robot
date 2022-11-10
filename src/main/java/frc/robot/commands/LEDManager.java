package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class LEDManager extends CommandBase {
    private  LEDs led;
        /** Creates a new VisionOff. */
        public LEDManager(final LEDs led) {
        this.led = led;
        addRequirements(led);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isDSAttached()) {
      if(DriverStation.isDisabled()) {
        led.rainbow();
      } else {
      
        if (DriverStation.getStickButton(0, Constants.LEDCelebrateButton)){
          led.superRainbow();
        } /*else if (DriverStation.getStickButton(0, Constants.ShooterButton)){
          led.shooterColorSpeed(Shooter.getRPM()); ///comment this if statement for competitions ie B3
        }*/else if (DriverStation.isAutonomous()){
          System.out.print(DriverStation.getAlliance());
            if (DriverStation.getAlliance() == Alliance.Blue){
              led.autonPulseBlue();
            }
            else if (DriverStation.getAlliance() == Alliance.Red){
              led.autonPulseRed();
            } else {
              led.setAllGreen();
            }
        } // end of auton
        else{     //if the robot is enabled 
           led.setAllGreen();}
      } // end of if not disabled
     } else {
      led.setAllOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
} 