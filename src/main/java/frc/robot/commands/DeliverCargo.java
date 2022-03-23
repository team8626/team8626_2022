// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Storage;
// Team8626 Libraries
import frc.robot.subsystems.StorageSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class DeliverCargo extends CommandBase {
  private final StorageSubsystem m_storage;

  /**
   * Creates a new DeliverCargo command.
   * This will empty the current storage toward the shooter.
   * Make sure the shooter is activated before scheduling this command.
   * 
   * @param storage The storage system to receive cargo from
   */
  public DeliverCargo(StorageSubsystem storage) {
    m_storage = storage;

    addRequirements(m_storage);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Unload the BACK UNIT
    new UnloadStorageUnit(m_storage.getBackUnit()).withTimeout(Storage.kTimeoutStorageUnit)
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    boolean ret_value = false;
    if(m_storage.isEmpty() == false) {
      ret_value = true;
    }
    return ret_value;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // TODO: Anything to stop or deactivate?
  }
}
