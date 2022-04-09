// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

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
public class StartCollectingCommand extends CommandBase {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem  m_intake;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new PrepareToCollectCommand command.
   * 
   * @param intake  The Intake
   * @param storage The Storage
   */
  public StartCollectingCommand(IntakeSubsystem intake, StorageSubsystem storage) {
    m_intake = intake;
    m_storage = storage;

    // addRequirements(m_intake);
    // addRequirements(m_storage);
    // addRequirements(m_storage.getFrontUnit());
    // addRequirements(m_storage.getBackUnit());
  }

  @Override
  public void initialize(){
    System.out.println("[StartCollectingCommand] Activate INTAKE");
    m_intake.activate();
  }

  @Override
  public void execute(){
    m_storage.loadCargo();
  }

  @Override
  public boolean isFinished() {
    boolean ret_value = false;
    // Storage is full or back to IDLE State (cancelled loading)
    if((m_storage.isFull() == true) || (m_storage.getStatus() == StorageSubsystem.Status.CANCELLED)) {
      ret_value = true;
    }
    return ret_value;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // The Storage will stop by itself
    // Deactivate the Intake
    System.out.println("[StartCollectingCommand] Deactivate INTAKE");
    m_intake.deactivate();
  }
}
