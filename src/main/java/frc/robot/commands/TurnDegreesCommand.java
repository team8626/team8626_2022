// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style. 
 * */
public class TurnDegreesCommand extends PIDCommand {
  private final DriveSubsystem m_drivetrain;
  private final DoubleSupplier m_angleDegrees;
  // private final Encoder m_leftEncoder;
  // private final Encoder m_rightEncoder;

  /**
   * Creates a new TankDriveCommand command.
   *
   *
   * @param angleDegrees 
  
   */
  
  public TurnDegreesCommand(DoubleSupplier angleDegrees, DriveSubsystem drivetrain) {

super( new PIDController(4, 0, 0), drivetrain::getHeading, angleDegrees, rot -> drivetrain.arcadeDrive(0, rot));

    m_drivetrain = drivetrain;
    m_angleDegrees = angleDegrees;
    

    // m_LeftSpeed = leftSpeed;
   // m_RightSpeed = rightSpeed;
  
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