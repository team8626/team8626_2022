// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.io.IOException;
import java.nio.file.Path;


// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward; 

// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveTrain;;

/**
 * Have the robot drive arcade style. 
 * */
public class FollowTrajectoryCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final DifferentialDriveVoltageConstraint m_voltageConstants;
  private final Trajectory m_trajectory;
  private final TrajectoryConfig m_trajectoryConfig;
  private final Path m_trajectoryPath;
  
  /**
   * Creates a new FollowTrajectoryCommand command.
   * @param filename JSON file containing the trajectory
   * @param drivetrain The drivetrain subsystem to drive
   * @throws IOException
   */
  public FollowTrajectoryCommand(String filename, DriveSubsystem drivetrain) throws IOException {
    m_drivetrain = drivetrain;

    // Read JSON files (deploy/filename) and Create  Trajectory
    m_trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
    m_trajectory = TrajectoryUtil.fromPathweaverJson(m_trajectoryPath);   
    
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

      // Reset odometry to the starting pose of the trajectory.
      m_drivetrain.resetOdometry(m_trajectory.getInitialPose());

      // Run path following command, then stop at the end.
      ramseteCommand.execute();
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // TODO: Under what condition is that finished ?
    // If current position is target position then return true
    return false; 
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
 