// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// Team8626 Libraries
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * Quick Autonomous mode routine
 * Shoot and drive backwards
 **/
public class CollectAndShootTwoCommand extends SequentialCommandGroup {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem  m_drivetrain;
  private final IntakeSubsystem m_intake;
  private final StorageSubsystem m_storage;
  private final ShooterSubsystem m_shooter;

  /**
   * Creates a new ShootAndMoveMetersCommand command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public CollectAndShootTwoCommand(double distanceMeters, DriveSubsystem drivetrain, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_storage = storage;
    m_shooter = shooter;

    addCommands(
        new SequentialCommandGroup(
            new PrepareToCollectCommand(m_intake, m_storage),
            new DriveMetersCommand(() -> distanceMeters, m_drivetrain),
            new StopCollectingCommand(m_intake, m_storage),
            new TurnDegreesCommand(() -> 180, m_drivetrain),
            new DriveMetersCommand(() -> distanceMeters, m_drivetrain),
            new LaunchCargoCommand(m_storage, m_shooter)
        )
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // Nothing to be done, all commands have timeouts
  }
}
