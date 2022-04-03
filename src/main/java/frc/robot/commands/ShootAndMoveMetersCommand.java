// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// Team8626 Libraries
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * Quick Autonomous mode routine
 * Shoot and drive backwards
 **/
public class ShootAndMoveMetersCommand extends SequentialCommandGroup {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem  m_drivetrain;
  private final ShooterSubsystem  m_shooter;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new ShootAndMoveMetersCommand command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public ShootAndMoveMetersCommand(double distanceMeters, DriveSubsystem drivetrain, StorageSubsystem storage, ShooterSubsystem shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_storage = storage;

    addCommands(
        new SequentialCommandGroup(
          new StartDeliveringCommand(m_storage, m_shooter),
          new DriveMetersCommand(() -> distanceMeters, m_drivetrain)
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
