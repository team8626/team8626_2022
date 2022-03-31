// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

// WPI Dependencies
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Team 8626 Dependencies
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.commands.PushCargo;
import frc.robot.commands.ShootAndMoveCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.PrepareToCollectCommand;
import frc.robot.commands.StopCollectingCommand;
import frc.robot.commands.LaunchCargoCommand;
import frc.robot.commands.ControlClimberCommand;
import frc.robot.commands.ControlStorageUnitCommand;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Storage;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final static StorageSubsystem m_storage = new StorageSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final static ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // private final ArcadeDriveCommand m_autoCommand = new ArcadeDriveCommand(m_DriveSubsystem);
 
  // Define controllers
  private final Joystick m_flightJoystick = new Joystick(Controller.kJoystickPort);
  private final XboxController m_gameController = new XboxController(Controller.kGamepadPort); 

  // Autonomous Mode
  private final static DashBoard m_dashboard = new DashBoard();
  private final static Autonomous m_autoControl = new Autonomous(m_dashboard, m_drivetrain, m_storage, m_shooter);
  // Intake is intially up
  private boolean isIntakeDown = false;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Set Default Commands for Subsystems if required...
   */
  private void configureDefaultCommands(){
    // Always push Cargo Forward....
    if(Storage.kIsUsingColorSensors){
      m_storage.setDefaultCommand(        
      new PushCargo(
        m_storage));
    }

     // Always Read Joystick and control the drivetrain
     m_drivetrain.setDefaultCommand(    
      new ArcadeDriveCommand(
        () -> -m_flightJoystick.getX(), 
        () -> m_flightJoystick.getY(),
        // () -> m_gameController.getRightY(), 
        // () -> m_gameController.getRightX(),
        m_drivetrain));

    // Always Read Joystick and control the climber arm
    // Note by default Climber arm is Disabled, will be enabled only when holding an extra button (see configureButtonBindings)
    m_climber.setDefaultCommand(        
      new ControlClimberCommand(
        () -> m_gameController.getRightY(),
        m_climber));
    //new InstantCommand(m_climber::setEnabled, m_climber);

    // Always Read Joystick and control the storage units
    // Front Storage Controlled by Left Joystick on Gamepad (X Axis)
    // Back Storage Controlled by Left Joystick on Gamepad (X Axis)
    m_storage.getFrontUnit().setDefaultCommand(        
      new ControlStorageUnitCommand(
        () -> -m_gameController.getLeftX(),
        m_storage.getFrontUnit()));

    m_storage.getBackUnit().setDefaultCommand(        
      new ControlStorageUnitCommand(
        () -> -m_gameController.getRightX(),
        m_storage.getBackUnit()));
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
    .whenPressed(new PrepareToCollectCommand(m_intake, m_storage));

    // Deactivate the intake Mechanism
    (new JoystickButton(m_gameController, Button.kB.value))
    .whenPressed(new StopCollectingCommand(m_intake, m_storage));

    // Flightstick Intake Activate/Deactivate
    if(m_flightJoystick.getTriggerPressed() && new JoystickButton(m_flightJoystick, 2).get()) {
      //Toggle intake
      isIntakeDown = !isIntakeDown;

      if(isIntakeDown) {
        new PrepareToCollectCommand(m_intake, m_storage);
         }
         if(!isIntakeDown) {
           new StopCollectingCommand(m_intake, m_storage);
            }
    }
    

    
    // Start Automatic Shooting Sequence
    // (new JoystickButton(m_gameController, Button.kStart.value))
    // .whenPressed(new LaunchCargoCommand(m_storage, m_shooter));

    // Start Manual Shooting
    (new JoystickButton(m_gameController, Button.kX.value))
    .whenPressed(new InstantCommand(m_shooter::activate, m_shooter));

    // Stop Manual Shooting
    (new JoystickButton(m_gameController, Button.kA.value))
    .whenPressed(new InstantCommand(m_shooter::deactivate, m_shooter));

    // Adjust Shooting power
    // (new JoystickButton(m_gameController, Button.kLeftBumper.value))
    //     .whenPressed(new InstantCommand(m_shooter::speedDown, m_shooter));
    // (new JoystickButton(m_gameController, Button.kRightBumper.value))
    //     .whenPressed(new InstantCommand(m_shooter::speedUp, m_shooter));
    
    
    // Climber Activated if Left Bumper is held. 
    // When Released, it will deactivate.
    // (new JoystickButton(m_gameController, Button.kStart.value))
    //    .whileHeld(new InstantCommand(m_climber::setEnabled, m_climber))
    //    .whenReleased(new InstantCommand(m_climber::setDisabled, m_climber));

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