// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPI Dependencies
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// Team 8626 Dependencies
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.commands.PushCargo;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.PrepareToCollect;
import frc.robot.commands.StopCollecting;
import frc.robot.commands.LaunchCargo;

import frc.robot.Constants.Controller;
import frc.robot.Constants.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final StorageSubsystem m_storage = new StorageSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // private final ArcadeDrive m_autoCommand = new ArcadeDrive(m_DriveSubsystem);
  // define controllers
  private final PS4Controller m_joystick = new PS4Controller(Controller.kPS4Port); 
  private final XboxController m_gameController = new XboxController(Controller.kGamepadPort); 



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default command for subsystems
    //
    m_storage.setDefaultCommand(        // Always push Cargo Forward....
      new PushCargo(
        m_storage));

    m_DriveSubsystem.setDefaultCommand( // Always Read Joystick and control the drivetrain
      new ArcadeDrive(
        () -> m_joystick.getLeftY(), 
        () -> m_joystick.getRightX(),
        m_DriveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Activate the intake Mecanism
    (new JoystickButton(m_gameController, Button.kY.value))
      .whenPressed(new PrepareToCollect(m_intake, m_storage));

    // Deactivate the intake Mechanism
   (new JoystickButton(m_gameController, Button.kB.value))
      .whenPressed(new StopCollecting(m_intake, m_storage));

    // Start Shooting Sequence
   (new JoystickButton(m_gameController, Button.kStart.value))
      .whenPressed(new LaunchCargo(m_storage, m_shooter));

  // TODO: Stop Shooting Sequence (Or make it stop automatically)
  //  (new JoystickButton(m_gameController, Button.kStart.value))
  //  .whenHeldPressed(new InstantCommand(m_climber, m_shooter));


  // TODO: Bind Climber to Buttons
    // Climber Activated if Left Bumper is held. 
    // Reads Right Stick on Gamepad for reading direction.
    // (new JoystickButton(m_gameController, Button.kLeftBumper.value))
    //   .whenHeld(new InstantCommand(m_climber::setPower, m_climber)
    //     () -> 1.0);
    // (new JoystickButton(m_gameController, Button.kLeftBumper.value))
    //   .whenPressed(new ConditionalCommand(m_climber::setPower(() -> 1.0), m_climber));
    //   ConditionalCommand(commandOnTrue, commandOnFalse, m_limitSwitch::get)
      //(m_climber::setPower, m_climber));

     

  // new JoystickButton(m_gameController, B)
  //   .and(new JoystickButton(exampleController, XboxController.Button.kY.value))
  //   .whenActive(new ExampleCommand());
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   // TODO: AUTONOMOUS COMMAND return m_autoCommand;

   // Create a voltage constraint to ensure we don't accelerate too fast
   var autoVoltageConstraint =
   new DifferentialDriveVoltageConstraint(
       new SimpleMotorFeedforward(
           DriveTrain.ksVolts,
           DriveTrain.kvVoltSecondsPerMeter,
           DriveTrain.kaVoltSecondsSquaredPerMeter),
           DriveTrain.kDriveKinematics,
       10);

// Create config for trajectory
TrajectoryConfig config =
   new TrajectoryConfig(
    DriveTrain.kMaxSpeedMetersPerSecond,
    DriveTrain.kMaxAccelerationMetersPerSecondSquared)
       // Add kinematics to ensure max speed is actually obeyed
       .setKinematics(DriveTrain.kDriveKinematics)
       // Apply the voltage constraint
       .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory =
   TrajectoryGenerator.generateTrajectory(
       // Start at the origin facing the +X direction
       new Pose2d(0, 0, new Rotation2d(0)),
       // Pass through these two interior waypoints, making an 's' curve path
       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
       // End 3 meters straight ahead of where we started, facing forward
       new Pose2d(3, 0, new Rotation2d(0)),
       // Pass config
       config);

RamseteCommand ramseteCommand =
   new RamseteCommand(
       exampleTrajectory,
       m_DriveSubsystem::getPose,
       new RamseteController(DriveTrain.kRamseteB, DriveTrain.kRamseteZeta),
       new SimpleMotorFeedforward(
           DriveTrain.ksVolts,
           DriveTrain.kvVoltSecondsPerMeter,
           DriveTrain.kaVoltSecondsSquaredPerMeter),
       DriveTrain.kDriveKinematics,
       m_DriveSubsystem::getWheelSpeeds,
       new PIDController(DriveTrain.kPDriveVel, 0, 0),
       new PIDController(DriveTrain.kPDriveVel, 0, 0),
       // RamseteCommand passes volts to the callback
       m_DriveSubsystem::tankDriveVolts,
       m_DriveSubsystem);

// Reset odometry to the starting pose of the trajectory.
m_DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> m_DriveSubsystem.tankDriveVolts(0, 0));
  }

  String trajectoryJSON = "paths/GoTerminal.wpilib.json";
  Trajectory terminalTrajectory = new Trajectory();

  // @Override
  // public void robotInit() {
  //   try {
  //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  //     terminalTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //   } 
  // }
}
