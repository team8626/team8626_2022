// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.PIDCommand;
// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive tank style. 
 * */
public class DriveStraightMetersCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;

  private double m_setPoint = 0;
  private int m_P, m_I, m_D = 1;
  private int m_integral_Left, m_integral_Right;
  private double m_previous_error_Left, m_previous_error_Right;
  private double m_power, m_powerLeft, m_powerRight = 0;

  /**
   * Creates a new TankDriveCommand command.
   *
   * @param distanceMeters 
   */
  
  public DriveStraightMetersCommand(DoubleSupplier distanceMeters, DoubleSupplier power, DriveSubsystem drivetrain) {
  
    m_setPoint = distanceMeters.getAsDouble();
    m_power = power.getAsDouble();
    m_drivetrain = drivetrain;
   // getController().setTolerance(0.01);
   System.out.println("[DRIVE_STRAIGHT] Start Driving "+ m_setPoint + "m");

    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }
  
  @Override
  public void execute()
  {
      PID();
      m_drivetrain.tankDrive(m_powerLeft * m_power, m_powerRight * m_power);
  }

  @Override
  public boolean isFinished() {
    boolean retval = false;

    // If each side is withing 1cm of target
    if((m_previous_error_Left < 0.01) || (m_previous_error_Right < 0.01)){
      System.out.println("[DRIVE_STRAIGHT] Done Driving");
      retval = true;
    }
    return retval;
  }

  private void PID(){
    double error = 0.0;
    double derivative;

    error = m_setPoint - m_drivetrain.getEncoderDistanceLeft(); // Error = Target - Actual
    m_integral_Left+= (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - m_previous_error_Left) / .02;
    m_powerLeft = m_P * error + m_I * m_integral_Left + m_D * derivative;
    m_previous_error_Left = error;

    error = m_setPoint - m_drivetrain.getEncoderDistanceRight(); // Error = Target - Actual
    m_integral_Right+= (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - m_previous_error_Right) / .02;
    m_powerRight = m_P * error + m_I * m_integral_Right + m_D * derivative;
    m_previous_error_Right = error;
  }

}