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
 * Have the robot store cargo style. 
 * */
public class StoreCargoCommand extends CommandBase {
  private final StorageSubsystem m_storage;

  /**
   * Creates a new StoreCargoCommand command.
   * This will start the storage unit until cancelled of cargo has been loaded.
   * 
   * @param storage The storage system to receive cargo from
   */
  public StoreCargoCommand(StorageSubsystem storage) {
    m_storage = storage;
    addRequirements(m_storage);
  }

  // Called Once when this Command is initialized
  @Override
  public void initialize() {
    new LoadStorageUnitCommand(m_storage.getFrontUnit());
  }

  @Override
  public boolean isFinished() {
    boolean ret_value = false;
    if(m_storage.isFull() == true) {
      ret_value = true;
    }
    return ret_value;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // Force stop of the storage units
    m_storage.getFrontUnit().stop();
    m_storage.getBackUnit().stop();
  }
}
