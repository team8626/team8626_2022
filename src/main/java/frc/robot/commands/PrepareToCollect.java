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
 * Get the robot ready to collect Cargo.
 *      - Get Intake Out
 *      - Start Loading the Front Storage Unit
 * 
 * If the Front Storage is already in use, this will do nothing.
 **/
public class PrepareToCollect extends ParallelCommandGroup {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem  m_intake;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new PrepareToCollect command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public PrepareToCollect(IntakeSubsystem intake, StorageSubsystem storage) {
    m_intake = intake;
    m_storage = storage;

    addCommands(
        // Activate the Intake
        new InstantCommand(m_intake::activate, m_intake),

        // Activate the Front Storage Unit
        new InstantCommand(m_storage::load, m_storage));
  }
}
