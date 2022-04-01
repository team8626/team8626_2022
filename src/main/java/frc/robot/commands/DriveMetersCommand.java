// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.PIDCommand;
// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style. 
 * */
public class DriveMetersCommand extends PIDCommand {
  private final DriveSubsystem m_drivetrain;

  /**
   * Creates a new TankDriveCommand command.
   *
   * @param distanceMeters 
   */
  
  public DriveMetersCommand(DoubleSupplier distanceMeters, DriveSubsystem drivetrain) {
    super(new PIDController(4, 0, 0), drivetrain::getAverageEncoderDistance, distanceMeters, d -> drivetrain.tankDrive(d, d));

    m_drivetrain = drivetrain;
    getController().setTolerance(0.01);

    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    m_drivetrain.zeroHeading();
    super.initialize();
  }
  
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}