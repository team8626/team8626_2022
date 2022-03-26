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
 * Have the robot drive arcade style. 
 * */
public class ArcadeDrive extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final double m_speed;
  private final double m_rotation;

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param speed The control input for the speed of the drive
   * @param rotation The control input for the rotation
   * @param drivetrain The drivetrain subsystem to drive
   */
  public ArcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_speed = speed.getAsDouble();
    m_rotation = rotation.getAsDouble();
    addRequirements(m_drivetrain);
  }

  public ArcadeDrive(double speed, double rotation, DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_rotation = rotation;
    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed, m_rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }
}
