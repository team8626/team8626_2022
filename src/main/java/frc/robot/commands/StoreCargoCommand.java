// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// Team8626 Libraries
import frc.robot.subsystems.StorageSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class StoreCargoCommand extends CommandBase {
  private final StorageSubsystem m_storage;

  /**
   * Creates a new LoadStorageUnit command.
   * This will start the storage unit until cancelled of cargo has been loaded.
   * 
   * @param storage The storage system to receive cargo from
   */
  public StoreCargoCommand(StorageSubsystem storage) {
    m_storage = storage;

    new LoadStorageUnitCommand(m_storage.getFrontUnit());
    addRequirements(m_storage);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Nothing here, executed once in the Command Constructor
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
    new InstantCommand(m_storage.getFrontUnit()::stop, m_storage.getFrontUnit());
    new InstantCommand(m_storage.getBackUnit()::stop, m_storage.getBackUnit());
  }
}
