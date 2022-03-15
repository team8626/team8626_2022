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
 * Have the robot drive arcade style. 
 * */
public class StoreCargo extends CommandBase {
  private final StorageSubsystem m_storage;
  //private final DoubleSupplier m_rotation;

  /**
   * Creates a new StoreCargo command.
   * 
   * @param storage The storage system to receive cargo from
   */
  public StoreCargo(StorageSubsystem storage) {
    m_storage = storage;
    addRequirements(m_storage);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // TODO: activate intake and get storage ready to accept cargo.
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // TODO: What is the finished condition
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // TODO: Anything to do here?
  }
}
