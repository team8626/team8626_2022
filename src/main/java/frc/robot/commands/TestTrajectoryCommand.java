// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.List;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveTrain;

/**
 * Have the robot drive arcade style. 
 * */
public class TestTrajectoryCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final DifferentialDriveVoltageConstraint m_voltageConstants;
  private final Trajectory m_trajectory;
  private final TrajectoryConfig m_trajectoryConfig;

  /**
   * Creates a new TestTrajectoryCommand command.
   * @param drivetrain The drivetrain subsystem to drive
   */
  public TestTrajectoryCommand(DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    m_voltageConstants = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          DriveTrain.ksVolts,
          DriveTrain.kvVoltSecondsPerMeter,
          DriveTrain.kaVoltSecondsSquaredPerMeter),
        DriveTrain.kDriveKinematics,
      DriveTrain.kMaxAvailableVoltage);

    // Create config for trajectory
    m_trajectoryConfig = new TrajectoryConfig(
      DriveTrain.kMaxSpeedMetersPerSecond,
      DriveTrain.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveTrain.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(m_voltageConstants);

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through the se two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RamseteCommand ramseteCommand =
    new RamseteCommand(
      m_trajectory,
      m_drivetrain::getPose,
      new RamseteController(DriveTrain.kRamseteB, DriveTrain.kRamseteZeta),
      new SimpleMotorFeedforward(
          DriveTrain.ksVolts,
          DriveTrain.kvVoltSecondsPerMeter,
          DriveTrain.kaVoltSecondsSquaredPerMeter),
      DriveTrain.kDriveKinematics,
      m_drivetrain::getWheelSpeeds,
      new PIDController(DriveTrain.kPDriveVel, 0, 0),
      new PIDController(DriveTrain.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drivetrain::tankDriveVolts,
      m_drivetrain); 

      ramseteCommand.execute();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if( false/*TODO: Find condition Finished trajectory*/ ) {
      return true;
    }
    return false;
  }
  
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
