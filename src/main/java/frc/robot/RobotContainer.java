// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final ClimberArms climberArms = new ClimberArms();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Feeder feeder = new Feeder(); 
  
  private final Vision vision = new Vision();

  private final LEDs leds = new LEDs();

  //private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final Joystick operatorStick = new Joystick(Constants.OperatorPort);
  private final Joystick leftStick = new Joystick(Constants.LeftDriverPort);
  private final Joystick rightStick = new Joystick(Constants.RightDriverPort);

// Intake Commands
  private final Command intakeCommand = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeCommand2 = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeStop = new RunCommand(
    () -> intake.stop(), intake);
  private final Command intakeReverse = new RunCommand(
    () -> intake.intake(-(Constants.IntakeSpeed)), intake);

// LED Commands
  private final Command ledDefault = new RunCommand(
    () -> leds.setAllGreen(), leds);
  private final Command ledRPMColor = new RunCommand(
    () -> leds.shooterColorSpeed(shooter.getRPM()), leds);
  
// Climber ARMS Commands
  private final Command climberArmsForward= new RunCommand(
    () -> climberArms.climbSide(Constants.ClimberArmsReverseSpeed), climberArms);
  private final Command climberArmsBack= new RunCommand(
    () -> climberArms.climbSide(Constants.ClimberArmsSpeed), climberArms);
  private final Command climberArmsStop = new RunCommand(
    () -> climberArms.stopSide(), climberArms);
    
// Climber MAIN Commands
  private final Command climberMainCommand = new RunCommand(
    () -> climber.climbMain(operatorStick.getY()), climber);
  private final Command climberStop = new RunCommand(
    () -> climber.stopMain(), climber);

// Feeder Commands 

  private final Command feederStop = new RunCommand(
    () -> feeder.stop(), feeder);
  private final Command feederCommand = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorSpeed), feeder);
  private final Command feederReverseCommand = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorReverseSpeed), feeder);
    private final Command feederReverseCommand2 = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorReverseSpeed), feeder);
  private final Command feederCommand2 = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorSpeed), feeder);

//Shooter Commands
  private final Command shooterStop = new RunCommand(
    () -> shooter.stop(), shooter);
  private final Command shooterCommand = new RunCommand(
    () -> shooter.shoot(), shooter);
  private final Command shooterCommand2 = new RunCommand(
    () -> shooter.shoot(), shooter);
  private final Command shooterCommand3 = new RunCommand(
    () -> shooter.shoot(), shooter);

// Drivetrain Commands
  private final Command tankDrive = new RunCommand(
    () -> drivetrain.tankDrive(-leftStick.getY(), -rightStick.getY()), drivetrain);
  private final Command getInRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));

// Vision Commands
  private final Command visionSystem = new RunCommand(
    () -> vision.turnLEDOn(), vision);
  private final Command visionOff = new VisionOff(vision);
  

// Autonomous Commands
  private final Command shoot = new ParallelCommandGroup(intakeCommand2, feederCommand2, shooterCommand2);
  private final Command driveBack = new DriveDistance(drivetrain, -30);
  private final Command driveBack2 = new DriveDistance(drivetrain, -10);
  private final Command autoShootDrive = new SequentialCommandGroup(
    driveBack.withTimeout(3), shooterCommand3.withTimeout(2), shoot.withTimeout(2), driveBack2.withTimeout(3));
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake.setDefaultCommand(intakeStop);

    climberArms.setDefaultCommand(climberArmsStop);

    climber.setDefaultCommand(climberStop);

    feeder.setDefaultCommand(feederStop);
    
    shooter.setDefaultCommand(shooterStop);

    drivetrain.setDefaultCommand(tankDrive);

    leds.setDefaultCommand(ledRPMColor);

    vision.setDefaultCommand(visionOff);

    //pdh.setSwitchableChannel(true);



    // Configure the button bindings
    configureButtonBindings();
    // start camera streaming
    CameraServer.startAutomaticCapture();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(operatorStick, Constants.FeederButton).whileHeld(feederCommand);

    new JoystickButton(operatorStick, Constants.FeederReverseButton).whileHeld(feederReverseCommand2);

    new JoystickButton(operatorStick, Constants.IntakeButton).whileHeld(feederReverseCommand);
    
    new JoystickButton(operatorStick, Constants.ShooterButton).whileHeld(shooterCommand);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileHeld(visionSystem);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonForward).whileHeld(climberArmsForward);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonBack).whileHeld(climberArmsBack);

    new JoystickButton(operatorStick, Constants.ClimberMainButtonDown).whileHeld(climberMainCommand);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileHeld(ledRPMColor);

    new JoystickButton(operatorStick, Constants.IntakeButton).whileHeld(intakeCommand);

    new JoystickButton(operatorStick, Constants.IntakeReverseButton).whileHeld(intakeReverse);

    new JoystickButton(leftStick, Constants.InRangeButton).whileHeld(getInRange);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  
    return autoShootDrive;
  }
}
