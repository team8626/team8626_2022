// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.StorageUnitSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class LoadStorageUnitCommand extends CommandBase {
  private final StorageUnitSubsystem m_storageUnit;

  /**
   * Creates a new LoadStorageUnitCommand command.
   * This will start the storage unit until cancelled of cargo has been loaded.
   * 
   * @param storage The storage system to receive cargo from
   */
  public LoadStorageUnitCommand(StorageUnitSubsystem storageUnit) {
    m_storageUnit = storageUnit;
    addRequirements(m_storageUnit);
  }

  // Called Once when this Command is started
  @Override
  public void initialize() {
    if(RobotBase.isSimulation()){ System.out.println("[LoadStorageUnitCommand] Starting " + m_storageUnit.getName()); }
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
    if(RobotBase.isSimulation()){ System.out.println("[LoadStorageUnit] Stopping " + m_storageUnit.getName()); }
    m_storageUnit.stop();
  }
}
