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
public class TurnDegreesCommand extends PIDCommand {
  private final DriveSubsystem m_drivetrain;

  /**
   * Creates a new TurnDegrees command.
   * @param angleDegrees 
   * @param drivetrain
   */
  
  public TurnDegreesCommand(DoubleSupplier angleDegrees, DriveSubsystem drivetrain) {

    super( new PIDController(4, 0, 0), drivetrain::getHeading, angleDegrees, rot -> drivetrain.arcadeDrive(0, rot));

    m_drivetrain = drivetrain;  
    addRequirements(m_drivetrain);
    
    getController().setTolerance(0.01);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void initialize() {
    m_drivetrain.zeroHeading();
    super.initialize();
  }
  
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}