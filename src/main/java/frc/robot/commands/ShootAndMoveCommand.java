// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Storage;
import frc.robot.subsystems.DriveSubsystem;
// Team8626 Libraries
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
//import frc.robot.commands.StoreCargo;

/**
 * Quick Autonomous mode routine
 * Shoot and drive backwards
 **/
public class ShootAndMoveCommand extends SequentialCommandGroup {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem  m_drivetrain;
  private final ShooterSubsystem  m_shooter;
  private final StorageSubsystem m_storage;

  private double m_driveSpeed = 1.0;
  private double m_driveRotation = 0.0;

  /**
   * Creates a new PrepareToCollect command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public ShootAndMoveCommand(DriveSubsystem drivetrain, StorageSubsystem storage, ShooterSubsystem shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_storage = storage;

    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(m_shooter::activate, m_shooter),
            new WaitCommand(Shooter.kShooterSpinSeconds),
            new UnloadStorageUnitCommand(m_storage.getFrontUnit())
             .withTimeout(Storage.kTimeoutStorageUnit),
                // new DeliverCargo(m_storage)
                //   .withTimeout(Storage.kTimeoutStorageUnit),

            //new InstantCommand(m_shooter::deactivate, m_shooter),

            // Drive Back until Timeout
            new TankDriveCommand(() -> m_driveSpeed, () -> -m_driveSpeed, m_drivetrain)
              .withTimeout(1.2)
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
