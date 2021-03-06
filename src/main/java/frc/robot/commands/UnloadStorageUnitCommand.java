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
 * Unload thge Storage Unit (Start Motor until timeout)
 **/
public class UnloadStorageUnitCommand extends CommandBase {
  private final StorageUnitSubsystem m_storageUnit;

  /**
   * Creates a new UnloadStorageUnitCommand command.
   * @param storage The storage system to receive cargo from
   */
  public UnloadStorageUnitCommand(StorageUnitSubsystem storageUnit) {
    m_storageUnit = storageUnit;
   
    addRequirements(m_storageUnit);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(RobotBase.isSimulation()){ System.out.println("[UnloadStorageUnitCommand] Starting " + m_storageUnit.getName()); }
    m_storageUnit.start();
  }

  @Override
  public boolean isFinished() {
    // Stops on Timeout...
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if(RobotBase.isSimulation()){ System.out.println("[UnloadStorageUnitCommand] Stopping " + m_storageUnit.getName()); }
    m_storageUnit.stop();
  }
}
