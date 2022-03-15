// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.StorageUnitSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class LoadStorageUnit extends CommandBase {
  private final StorageUnitSubsystem m_storageUnit;

  /**
   * Creates a new PushCargo command.
   * This will push cargo from front unit to back unit.
   * If Back unit is already in use, nothing will happen.
   * 
   * @param storage The storage system to receive cargo from
   */
  public LoadStorageUnit(StorageUnitSubsystem storageUnit) {
    m_storageUnit = storageUnit;

    addRequirements(m_storageUnit);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_storageUnit.start();
  }

  @Override
  public boolean isFinished() {
    boolean ret_value = false;
    if(m_storageUnit.isEmpty() == false) {
      ret_value = true;
    }
    return ret_value;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_storageUnit.stop();
  }
}
