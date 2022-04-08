// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Java Libraries

// WPI Library dependencies

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
public class StopCollectingCommand extends CommandBase {
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
  }

  @Override
  public void initialize(){
    System.out.println("[StopCollectingCommand] Deactivate INTAKE");
    m_intake.deactivate();
    System.out.println("[StopCollectingCommand] Aboard Loading"); 
    m_storage.stopLoadingCargo();
  }

  @Override
  public void execute(){}

  @Override
  public boolean isFinished(){
    // Only execute a few methods at initialization.
    // Finishes immediately.
    return true;
  }

  @Override
  public void end(boolean interrupted){}

}
