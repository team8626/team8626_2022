// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style. 
 * */
public class TankDriveCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final DoubleSupplier m_LeftSpeed;
  private final DoubleSupplier m_RightSpeed;

  /**
   * Creates a new TankDriveCommand command.
   *
   * @param speed The control input for the speed of the drive
   * @param leftSpeed Left Side Speed
   * @param rightSpeed Right Side Speed
   */
  public TankDriveCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_LeftSpeed = leftSpeed;
    m_RightSpeed = rightSpeed;
    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_LeftSpeed.getAsDouble(), m_RightSpeed.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0);
  }
}
