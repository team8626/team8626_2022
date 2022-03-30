// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// Team8626 Libraries
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * Stop the Cargo Collecting
 *      - Get Intake Up
 *      - Force stop Front Storage Unit
 * 
 * If the Front Storage is already in use, this will do nothing.
 **/
public class StopCollectingCommand extends ParallelCommandGroup {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem  m_intake;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new StopCollectingCommand command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public StopCollectingCommand(IntakeSubsystem intake, StorageSubsystem storage) {
    m_intake = intake;
    m_storage = storage;

    addCommands(
        // Deactivate the Intake
        new InstantCommand(m_intake::deactivate, m_intake),

        // Deactivate the Storage Unit
        new InstantCommand(m_storage.getFrontUnit()::stop, m_storage.getFrontUnit()));
  }
}
