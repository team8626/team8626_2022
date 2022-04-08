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
  
  public TurnDegreesCommand(DoubleSupplier angleDegrees, DoubleSupplier power, DriveSubsystem drivetrain) {

    super( new PIDController(4, 0, 0), drivetrain::getHeading, angleDegrees, rot -> drivetrain.arcadeDrive(0, rot * power.getAsDouble()));

    m_drivetrain = drivetrain;  
    addRequirements(m_drivetrain);
    
    System.out.println("[TURN_DEGREES] Start Turning "+ angleDegrees.getAsDouble() + "deg");

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
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
    System.out.println("[TURN_DEGREES] Done Turning ");
    return getController().atSetpoint();
  }
}