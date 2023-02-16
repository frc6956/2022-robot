// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.SetSideArmPosition;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
//import com.ctre.phoenixpro.hardware.Pigeon;
//import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


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
  //private Gyro m_gyro;
  private final Vision vision = new Vision();
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  public final Drivetrain drivetrain = new Drivetrain(m_gyro, vision);
  private final Feeder feeder = new Feeder();
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
  private final Command intakeCommand3 = new RunCommand(
    () -> intake.intake(Constants.IntakeSpeed), intake);
  private final Command intakeCommand7 = new RunCommand(
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
    /* 
  private final Command climberArmsHold = new RunCommand(
    () -> climberArms.hold(), climberArms);*/
  private final Command climberArmsStop = new RunCommand(
    () -> climberArms.stopSide(), climberArms);

  private final Command setSideArmsHigh = new SetSideArmPositionHigh(climberArms);
  private final Command setSideArmsMid = new SetSideArmPositionMid(climberArms);
  private final Command setSideArmsBack = new SetSideArmPositionBack(climberArms);

  final Command resetArms = new ResetArms(climberArms);
    
// Climber MAIN Commands
  private final Command climberMainCommand = new RunCommand(
    () -> climber.climbMain(operatorStick.getY()), climber);
  private final Command climberStop = new RunCommand(
    () -> climber.stopMain(), climber);
  private final Command climberLeftCommand = new RunCommand(
    () -> climber.climbLeft(operatorStick.getY()), climber);
  private final Command climberRightCommand = new RunCommand(
    () -> climber.climbRight(operatorStick.getY()), climber);

  private final Command  setArmPositionMid = new SetArmPositionMid(climber);
  private final Command  setArmPositionHigh = new SetArmPositionHigh(climber);
  private final Command  returnArmPosition = new ReturnArmPosition(climber);

  private final Command  setSideArmPositionMid = new SetSideArmPositionMid(climberArms);
  private final Command  setSideArmPositionHigh = new SetSideArmPositionHigh(climberArms);
//  private final Command  returnSideArmPosition = new ReturnSideArmPosition(climberArms);
  
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
  private final Command feederCommand3 = new RunCommand(
    () -> feeder.feed(Constants.FeederMotorSpeed), feeder);

//Shooter Commands
  private final Command shooterStop = new RunCommand(
    () -> shooter.stop(), shooter);
  private final Command shooterCommand = new RunCommand(
    () -> shooter.shoot(), shooter);
  private final Command shooterLowCommand = new RunCommand(
    () -> shooter.shootLow(), shooter);
  private final Command shooterCommand2 = new RunCommand(
    () -> shooter.autoShoot(), shooter);
  private final Command shooterCommand3 = new RunCommand(
    () -> shooter.autoShoot(), shooter);

// Drivetrain Commands
  private final Command tankDrive = new RunCommand(
    () -> drivetrain.tankDrive(-leftStick.getY(), -rightStick.getY()), drivetrain);
  private final Command getInAngleRange = new RunCommand(
    () -> drivetrain.getInAngleRange(vision.getX()-5));
  private final Command getInDistanceRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));
  private final Command getInRobotRange = new RunCommand(
    () -> drivetrain.getInAllRange(vision.getDistance(), vision.getX()));
   

  // Vision Commands
  private final Command visionOn = new RunWhenDisabledCommand(() -> vision.turnLEDOn(), vision);
  private final Command visionOff = new RunWhenDisabledCommand(() -> vision.turnLEDOff(), vision);
  private final Command visionApril = new RunCommand(() -> vision.setAprilTags(), vision);
  private final Command visionLimelight = new RunCommand(() -> vision.setLimelight(), vision);
  

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
  //private final Command turnRobot = new TurnAngle(drivetrain, gyro, 180);
  private final Command driveBack3 = new DriveDistance(drivetrain, -20);
  private final Command driveBack4 = new DriveDistance(drivetrain, -20, 0.6);
  private final Command getAutoInRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));
  private final Command getAutoAngleRange = new RunCommand(
    () -> drivetrain.getInAngleRange(vision.getX()));
  private final Command climberArmsAuto2 = new RunCommand(
    () -> climberArms.climbSide(0), climberArms);
  private final Command climberArmsPATHTEST = new RunCommand(
    () -> climberArms.climbSide(0.5), climberArms);

  private final Command shooterCommand7357 = new RunCommand(
    () -> shooter.autoShoot(), shooter);
  
  private final Command auto2BallDrive = new SequentialCommandGroup(climberArmsAuto2.withTimeout(.1),
    intakeForward.withTimeout(10), shootStop.withTimeout(0.1), getAutoAngleRange.withTimeout(1), 
    getAutoInRange.withTimeout(4), shooterCommand6.withTimeout(1.5),shoot2.withTimeout(1.5), shootStop2.withTimeout(0.1), driveBack4.withTimeout(3));

  private final Command turnAngle = new TurnAngle(drivetrain, m_gyro, 180);  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  String trajectoryJSON = "/home/lvuser/deploy/output/TestMatch.wpilib.json";
  Trajectory testTrajectory = new Trajectory();
 
  String trajectoryCJSON = "/home/lvuser/deploy/output/Complex.wpilib.json";
  Trajectory complexTrajectory = new Trajectory();

  String autonID1JSON = "/home/lvuser/deploy/output/AutonID1.wpilib.json";
  Trajectory autonID1Trajectory = new Trajectory();

  String autonID2JSON = "/home/lvuser/deploy/output/AutonID2.wpilib.json";
  Trajectory autonID2Trajectory = new Trajectory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   // m_chooser.setDefaultOption("2 Ball Auto", auto2BallDrive);
    m_chooser.addOption("1 Ball Auto", autoShootDrive);
  
    SmartDashboard.putData(m_chooser);

    
  

    SmartDashboard.putData(m_gyro);

    intake.setDefaultCommand(intakeStop);

    climberArms.setDefaultCommand(climberArmsStop);

    climber.setDefaultCommand(climberStop);

    feeder.setDefaultCommand(feederStop);
    
    shooter.setDefaultCommand(shooterStop);

    drivetrain.setDefaultCommand(tankDrive);

    vision.setDefaultCommand(visionApril);

    leds.setDefaultCommand(ledManager);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    try {
      Path trajectoryComplexPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryCJSON);
      complexTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryComplexPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryCJSON, ex.getStackTrace());
    }

    try {
      Path autonID1Path = Filesystem.getDeployDirectory().toPath().resolve(autonID1JSON);
      autonID1Trajectory = TrajectoryUtil.fromPathweaverJson(autonID1Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + autonID1JSON, ex.getStackTrace());
    }

    try {
      Path autonID2Path = Filesystem.getDeployDirectory().toPath().resolve(autonID2JSON);
      autonID2Trajectory = TrajectoryUtil.fromPathweaverJson(autonID2Path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + autonID2JSON, ex.getStackTrace());
    }
/* 
    try {
      Path trajectoryBackPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      testBackTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryBackPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    try {
      Path backTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(backJSON);
      backTrajectory = TrajectoryUtil.fromPathweaverJson(backTrajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + backJSON, ex.getStackTrace());
    }

    try {
      Path aprilPath = Filesystem.getDeployDirectory().toPath().resolve(backJSON);
      aprilTrajectory = TrajectoryUtil.fromPathweaverJson(aprilPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + backJSON, ex.getStackTrace());
    }

    try {
      Path aprilBackPath = Filesystem.getDeployDirectory().toPath().resolve(backJSON);
      aprilBackTrajectory = TrajectoryUtil.fromPathweaverJson(aprilBackPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + backJSON, ex.getStackTrace());
    } */

    // Configure the button bindings
    configureButtonBindings();
    // start camera streaming
    CameraServer.startAutomaticCapture();

    SmartDashboard.putData(drivetrain);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(operatorStick, Constants.FeederButton).whileTrue(feederCommand);

    new JoystickButton(operatorStick, Constants.FeederReverseButton).whileTrue(feederReverseCommand2);

    new JoystickButton(operatorStick, Constants.IntakeButton).whileTrue(feederReverseCommand);

    new JoystickButton(operatorStick, Constants.FeederButton).whileTrue(intakeCommand3);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileTrue(shooterCommand);

    new JoystickButton(operatorStick, Constants.ShooterButton).whileTrue(visionOn);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonForward).whileTrue(climberArmsForward);

    new JoystickButton(operatorStick, Constants.ClimberArmsButtonBack).whileTrue(climberArmsBack);

    new JoystickButton(operatorStick, Constants.ClimberMainButtonDown).whileTrue(climberMainCommand);

    new JoystickButton(operatorStick, Constants.ClimberLeftMainButton).whileTrue(climberLeftCommand);

    new JoystickButton(operatorStick, Constants.ClimberRightMainButton).whileTrue(climberRightCommand);

    new JoystickButton(operatorStick, Constants.IntakeButton).whileTrue(intakeCommand);

    new JoystickButton(operatorStick, Constants.IntakeReverseButton).whileTrue(intakeReverse);

    new JoystickButton(leftStick, Constants.ApriltagButton).whileTrue(visionApril);

    new JoystickButton(leftStick, Constants.VisionTapeButton).whileTrue(visionLimelight);

    new JoystickButton(leftStick, Constants.InRangeButton).whileTrue(getInDistanceRange);

    new JoystickButton(rightStick, Constants.InRangeButton).whileTrue(getInAngleRange);

    new JoystickButton(operatorStick, Constants.ShooterLowButton).whileTrue(shooterLowCommand);

    new JoystickButton(operatorStick, Constants.ShooterLowButton).whileTrue(intakeCommand7);

    new JoystickButton(operatorStick, Constants.ShooterLowButton).whileTrue(feederCommand3);

    new JoystickButton(rightStick, Constants.ClimberMainPosition).whileTrue(setArmPositionMid);

    new JoystickButton(rightStick, Constants.ClimberMainPositionHigh).whileTrue(setArmPositionHigh);

    new JoystickButton(rightStick, Constants.ClimberMainReturnPosition).whileTrue(returnArmPosition);

    new JoystickButton(leftStick, Constants.ClimberMainReturnPosition).whileTrue(setSideArmsHigh);

    new JoystickButton(leftStick, Constants.ClimberMainPositionHigh).whileTrue(setSideArmsMid);

    new JoystickButton(leftStick, Constants.ClimberMainPosition).whileTrue(setSideArmsBack);


    new Trigger(()-> DriverStation.isAutonomous()).whileTrue(visionApril);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand1() {
    
    return m_chooser.getSelected();
    
  }

  public Command getAutonomousCommandPath() {
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);

    TrajectoryConfig forwardConfig =
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(false);

    TrajectoryConfig backwardConfig =
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);
    


    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          List.of(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            new Pose2d(2, 1, new Rotation2d(0)),
            new Pose2d(4, -1, new Rotation2d(0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0))
          ),
          // Pass config
          forwardConfig);

    //Subsystem mainRobotDrive;
    RamseteCommand testRamseteCommand =
      new RamseteCommand(
        testTrajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel+.1, 0, 0),
          new PIDController(Constants.kPDriveVel+.1, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);

    RamseteCommand testComplexRamseteCommand =
      new RamseteCommand(
        complexTrajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);
    
    RamseteCommand autonID1Command =
      new RamseteCommand(
        autonID1Trajectory,
            drivetrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel-0, 0.0, 0.0),
            new PIDController(Constants.kPDriveVel-0, 0.0, 0.0),
              // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);
        
    RamseteCommand autonID2Command =
      new RamseteCommand(
        autonID2Trajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
           // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);

/* 
    RamseteCommand backRamseteCommand =
      new RamseteCommand(
        backTrajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);


    RamseteCommand testNewBackRamseteCommand =
      new RamseteCommand(
        testTrajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain
          );

    RamseteCommand aprilRamseteCommand =
      new RamseteCommand(
        aprilTrajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);

      RamseteCommand aprilBackRamseteCommand =
          new RamseteCommand(
            aprilBackTrajectory,
              drivetrain::getPose,
              new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
              new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
              drivetrain::getWheelSpeeds,
              new PIDController(Constants.kPDriveVel, 0, 0),
              new PIDController(Constants.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              drivetrain::tankDriveVolts,
              drivetrain); */

  // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(aprilBackTrajectory.getInitialPose());
    //drivetrain.resetOdometry(vision.getLimelightPose());
    

    

    //final Command robotTrajectory = new SequentialCommandGroup(testRamseteCommand, testBackRamseteCommand);

  // Run path following command, then stop at the end.
    if (vision.getAprilID() == 3.00){
      return testComplexRamseteCommand.andThen(setArmPositionHigh);
    } else if(vision.getAprilID() == 2.00){
      return autonID2Command.andThen(setArmPositionMid);
    } else if(vision.getAprilID() == 1.00){
      return autonID1Command.andThen(turnAngle).andThen(autonID2Command).andThen(setArmPositionMid).andThen(setArmPositionHigh).andThen(setSideArmsMid).andThen(setSideArmsHigh);
    } else{
      return testComplexRamseteCommand.andThen(() -> drivetrain.tankDrive(0, 0));
    }


    //return aprilBackRamseteCommand.andThen(aprilRamseteCommand).andThen(() -> drivetrain.tankDrive(0, 0));
    //return testRamseteCommand.andThen( () -> drivetrain.resetOdometry(testBackTrajectory.getInitialPose())).andThen(testBackRamseteCommand);
  }
   
}

