// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
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
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);

  //private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final Joystick operatorStick = new Joystick(Constants.OperatorPort);
  private final Joystick leftStick = new Joystick(Constants.LeftDriverPort);
  private final Joystick rightStick = new Joystick(Constants.RightDriverPort);

// Intake Commands
  private final Command intakeCommand = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeCommand2 = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeCommand3 = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeStop = new RunCommand(
    () -> intake.stop(), intake);
  private final Command intakeReverse = new RunCommand(
    () -> intake.intake(-(Constants.IntakeSpeed)), intake);

// LED Commands
    private final Command ledManager = new LEDManager(leds);
  
// Climber ARMS Commands
  private final Command climberArmsForward= new RunCommand(
    () -> climberArms.climbSide(Constants.ClimberArmsReverseSpeed), climberArms);
  private final Command climberArmsBack= new RunCommand(
    () -> climberArms.climbSide(Constants.ClimberArmsSpeed), climberArms);
  private final Command climberArmsStop = new RunCommand(
    () -> climberArms.hold(), climberArms);
    
// Climber MAIN Commands
  private final Command climberMainCommand = new RunCommand(
    () -> climber.climbMain(operatorStick.getY()), climber);
  private final Command climberStop = new RunCommand(
    () -> climber.stopMain(), climber);
  private final Command climberLeftCommand = new RunCommand(
    () -> climber.climbLeft(operatorStick.getY()), climber);
  private final Command climberRightCommand = new RunCommand(
    () -> climber.climbRight(operatorStick.getY()), climber);
  

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
    () -> shooter.autoShoot(), shooter);
  private final Command shooterCommand3 = new RunCommand(
    () -> shooter.autoShoot(), shooter);

// Drivetrain Commands
  private final Command tankDrive = new RunCommand(
    () -> drivetrain.tankDrive(-leftStick.getY(), -rightStick.getY()), drivetrain);
  private final Command getInAngleRange = new RunCommand(
    () -> drivetrain.getInAngleRange(vision.getX()));
  private final Command getInDistanceRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));
  private final Command getInAllRange = new RunCommand(
    () -> drivetrain.getInAllRange(vision.getDistance(), vision.getX()));
   

  // Vision Commands
  private final Command visionOn = new RunCommand(() -> vision.turnLEDOn(), vision);
  private final Command visionOff = new RunWhenDisabledCommand(() -> vision.turnLEDOff(), vision);
  

// Autonomous Commands
  private final Command shoot = new ParallelCommandGroup(intakeCommand2, feederCommand2, shooterCommand2);
  private final Command driveBack = new DriveDistance(drivetrain, -10);
  private final Command driveBack2 = new DriveDistance(drivetrain, -30);
  private final Command climberArmsAuto = new RunCommand(
    () -> climberArms.climbSide(0), climberArms);
  private final Command autoShootDrive = new SequentialCommandGroup(climberArmsAuto.withTimeout(.1),
    driveBack.withTimeout(3), shooterCommand3.withTimeout(2), shoot.withTimeout(2), driveBack2.withTimeout(3));

    // Auto 2 Ball
  private final Command shooterCommand5 = new RunCommand(
    () -> shooter.autoShoot(), shooter);
  private final Command shooterCommand6 = new RunCommand(
    () -> shooter.autoShoot(), shooter);
  private final Command feederCommand5 = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorSpeed), feeder);

  private final Command feederReverseCommand3 = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorReverseSpeed), feeder);
  
  private final Command intakeCommand5 =  new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeCommand6 =  new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command shoot2 = new ParallelCommandGroup(intakeCommand5, feederCommand5, shooterCommand5);

  private final Command feederStop2 = new RunCommand(
    () -> feeder.stop(), feeder);
  private final Command intakeStop2 = new RunCommand(
    () -> intake.stop(), intake); 
  private final Command feederStop3 = new RunCommand(
    () -> feeder.stop(), feeder);
  private final Command intakeStop3 = new RunCommand(
    () -> intake.stop(), intake); 

  private final Command driveForward = new DriveDistance(drivetrain, 55); 
  private final Command intakeForward = new ParallelRaceGroup(intakeCommand6, feederReverseCommand3, driveForward);
  private final Command shootStop = new ParallelCommandGroup(feederStop2, intakeStop2);
  private final Command shootStop2 = new ParallelCommandGroup(feederStop3, intakeStop3);
  private final Command autoTurn = new TurnDistance(drivetrain, 40);
  private final Command turnRobot = new TurnAngle(drivetrain, gyro, 175);
  private final Command driveBack3 = new DriveDistance(drivetrain, -20);
  private final Command driveBack4 = new DriveDistance(drivetrain, -20);
  private final Command getAutoInRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));
  private final Command getAutoAngleRange = new RunCommand(
    () -> drivetrain.getInAngleRange(vision.getX()));
  private final Command climberArmsAuto2 = new RunCommand(
    () -> climberArms.climbSide(0), climberArms);
  
  private final Command auto2BallDrive = new SequentialCommandGroup(climberArmsAuto2.withTimeout(.1),
    intakeForward.withTimeout(10), shootStop.withTimeout(0.1), turnRobot.withTimeout(6), getAutoAngleRange.withTimeout(3), 
    getAutoInRange.withTimeout(4), shooterCommand6.withTimeout(1.5),shoot2.withTimeout(1.5), shootStop2.withTimeout(0.1), driveBack4.withTimeout(3));

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_chooser.setDefaultOption("1 Ball Auto", autoShootDrive);
    m_chooser.addOption("2 Ball Auto", auto2BallDrive);
  
    SmartDashboard.putData(m_chooser);

    SmartDashboard.putData(gyro);

    intake.setDefaultCommand(intakeStop);

    climberArms.setDefaultCommand(climberArmsStop);

    climber.setDefaultCommand(climberStop);

    feeder.setDefaultCommand(feederStop);
    
    shooter.setDefaultCommand(shooterStop);

    drivetrain.setDefaultCommand(tankDrive);

    vision.setDefaultCommand(visionOff);

    leds.setDefaultCommand(ledManager);



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

    new JoystickButton(operatorStick, Constants.FeederButton).whileHeld(intakeCommand3);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileHeld(shooterCommand);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileHeld(visionOn);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonForward).whileHeld(climberArmsForward);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonBack).whileHeld(climberArmsBack);

    new JoystickButton(operatorStick, Constants.ClimberMainButtonDown).whileHeld(climberMainCommand);

    new JoystickButton(operatorStick, Constants.ClimberLeftMainButton).whileHeld(climberLeftCommand);

    new JoystickButton(operatorStick, Constants.ClimberRightMainButton).whileHeld(climberRightCommand);

    new JoystickButton(operatorStick, Constants.IntakeButton).whileHeld(intakeCommand);

    new JoystickButton(operatorStick, Constants.IntakeReverseButton).whileHeld(intakeReverse);

    new JoystickButton(leftStick, Constants.InRangeButton).whileHeld(getInDistanceRange);

    new JoystickButton(rightStick, Constants.InRangeButton).whileHeld(getInAllRange);

    new JoystickButton(leftStick, Constants.InRangeButton).whileHeld(visionOn);

    new JoystickButton(rightStick, Constants.InRangeButton).whileHeld(visionOn);


    new Trigger(()-> DriverStation.isAutonomous()).whileActiveContinuous(visionOn);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  
    return m_chooser.getSelected();
    
  }
}
