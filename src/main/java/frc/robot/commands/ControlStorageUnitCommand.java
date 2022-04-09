// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.StorageUnitSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class ControlStorageUnitCommand extends CommandBase {
  private final StorageUnitSubsystem m_storageUnit;
  private final DoubleSupplier m_speed;

  /**
   * Creates a new Climb command.
   *
   * @param speed The control input for the speed of the climber
   * @param climber The climber subsystem to drive
   */
  public ControlStorageUnitCommand(DoubleSupplier speed, StorageUnitSubsystem storageUnit) {
    m_storageUnit = storageUnit;
    m_speed = speed;
    addRequirements(m_storageUnit);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_storageUnit.setPower(m_speed.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; 
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
