// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPI Dependencies
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;

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

import frc.robot.DashBoard;
import frc.robot.Autonomous;

import frc.robot.Constants.Controller;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final StorageSubsystem m_storage = new StorageSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // private final ArcadeDrive m_autoCommand = new ArcadeDrive(m_DriveSubsystem);
 
  // Define controllers
  private final Joystick m_flightJoystick = new Joystick(Controller.kJoystickPort);
  private final XboxController m_gameController = new XboxController(Controller.kGamepadPort); 

  // Autonomous Mode
  private final DashBoard m_dashboard = new DashBoard();
  private final Autonomous m_autoControl = new Autonomous(m_dashboard, m_drivetrain);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    // Set default command for subsystems
    //
    m_storage.setDefaultCommand(        // Always push Cargo Forward....
      new PushCargo(
        m_storage));

    m_drivetrain.setDefaultCommand(     // Always Read Joystick and control the drivetrain
      new ArcadeDrive(
        () -> m_flightJoystick.getY(), 
        () -> m_flightJoystick.getX(),
        m_drivetrain));
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

    if (m_flightJoystick.getRawButtonPressed(2)) {
      //setLowSpeed(); // TODO:  When held low speed mode activates
    }

    if (m_flightJoystick.getRawButtonReleased(2)) {
      //setHighSpeed(); // TOOD: When released the speed scales back to its normal high speed
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
    Command retval = null;
    try {
      retval = m_autoControl.getStartCommand();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return retval;
  }
}
