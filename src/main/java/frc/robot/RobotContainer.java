// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPI Dependencies
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

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
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.PushCargo;
import frc.robot.commands.ArcadeDrive;
import frc.robot.Constants.Controller;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Dashboard;
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

  // private final ArcadeDrive m_autoCommand = new ArcadeDrive(m_DriveSubsystem);
  // define controllers
  Joystick m_flightJoystick = new Joystick(Controller.kPS4Port);
  // private final PS4Controller m_joystick = new PS4Controller(Controller.kPS4Port); 

  // Autonomous Mode
  private final DashBoard m_dashboard = new Dashboard();
  private final Autonomous m_autoControl = new Autonomous(m_dashboard, m_drivetrain);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // set default command for subsystems
    m_storage.setDefaultCommand(
      new PushCargo(
        m_storage));
    m_DriveSubsystem.setDefaultCommand(
      new ArcadeDrive(
        () -> m_flightJoystick.getY(), 
        () -> m_flightJoystick.getX(),
        () -> m_flightJoystick.getZ(),
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
  
   public class JoystickButton extends Button {
   JoystickButton ButtonTwo = new JoystickButton(m_flightJoystick, 2);
   
  }

  private void configureButtonBindings() {

    if (m_flightJoystick.getRawButtonPressed(2)) {
      setLowSpeed(); // When held low speed mode activates
    }

    if (m_flightJoystick.getRawButtonReleased(2)) {
      setHighSpeed(); // When released the speed scales back to its normal high speed
    }

    // TODO: Bind Buttons to Shooting
    // if (m_flightJoystick.getTriggerPressed() && m_flightJoystick.getRawButtonPressed(3)) {
    //   setDefaultCommand =
    // }
  }

  /**
   * Get Start command from the autonomous controller (Dashboard)
   */
  public Command getAutonomousCommand() {
    return m_autoControl.getStartCommand();
  }
 


}
