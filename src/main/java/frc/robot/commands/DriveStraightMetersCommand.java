// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style. 
 * */
public class DriveStraightMetersCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;

  private double m_setPoint = 0;
  private double m_P = 0.001;
  private double m_I = 0;
  private double m_D = 0;
  private double m_powerRatio;
  private PIDController m_LeftController;
  private PIDController m_RightController;

  /**
   * Creates a new DriveStraightMetersCommand command.
   *
   * @param distanceMeters Distance to be driven
   * @param power Power Ratio to be applie to the driving [0-1]
   */
  
  public DriveStraightMetersCommand(DoubleSupplier distanceMeters, DoubleSupplier power, DriveSubsystem drivetrain) {
  
    m_setPoint = distanceMeters.getAsDouble();
    m_powerRatio = power.getAsDouble();
    m_drivetrain = drivetrain;

    m_LeftController = new PIDController(m_P, m_I, m_D);
    m_RightController = new PIDController(m_P, m_I, m_D);
    
    System.out.println("[DRIVE_STRAIGHT] Start Driving "+ m_setPoint + "m");

    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    m_LeftController.setSetpoint(m_setPoint);
    m_LeftController.setTolerance(0.01);
    m_LeftController.setIntegratorRange(-1.0, 1.0);
    
    m_RightController.setSetpoint(m_setPoint);
    m_RightController.setTolerance(0.01);
    m_RightController.setIntegratorRange(-1.0, 1.0);
  }
  
  @Override
  public void execute()
  {
      // SketchyPID();
      double powerLeft  = m_LeftController.calculate(m_drivetrain.getEncoderDistanceLeft());
      double powerRight = m_RightController.calculate(m_drivetrain.getEncoderDistanceRight());
      
      m_drivetrain.tankDrive(powerLeft * m_powerRatio, powerRight * m_powerRatio);
  }

  @Override
  public boolean isFinished() {
    boolean retval = false;

    // If both sides are at setpoint
    if((m_LeftController.atSetpoint() && m_RightController.atSetpoint())){
      System.out.println("[DRIVE_STRAIGHT] Done Driving");
      retval = true;
    }
    return retval;
  }
}