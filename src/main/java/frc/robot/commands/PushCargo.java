// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.StorageSubsystem;

/**
 * This command will push cargo from front unit to back unit.
 * If Back unit is already in use, nothing will happen.
 * */
public class PushCargo extends CommandBase {
  private final StorageSubsystem m_storage;

  /**
   * Creates a new PushCargo command.
   * 
   * @param storage The storage system to receive cargo from
   */
  public PushCargo(StorageSubsystem storage) {
    m_storage = storage;

    addRequirements(m_storage);
  }

  // Called repeatedly when this Command is scheduled to run
  // TODO: Use PerpetualCommand ???
  @Override
  public void execute() {
    m_storage.storeForward();
  }

  @Override
  public boolean isFinished() {
    return false; // Never Stops, always try to push Cargo...
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // TODO: Anything to stop or deactivate?
  }
}
