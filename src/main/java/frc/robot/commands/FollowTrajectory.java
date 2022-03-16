// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// Team8626 Libraries
import frc.robot.subsystems.DriveSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class FollowTrajectory extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final Trajectory m_trajectory;
  private final TrajectoryConfig m_config;
  private final DifferentialDriveVoltageConstraint m_voltageConstants;
  // private final Trajectory m_trajectory;
  // private final TrajectoryConfig m_trajectoryConfig;

  /**
   * Creates a new ArcadeDrive command.
   * @param filename JSON file containing the trajectory
   * @param drivetrain The drivetrain subsystem to drive
   */
  public FollowTrajectory(string filename, DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;

    // Read JSON files (deploy/filename) and Create  Trajectory
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
    m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);   
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    m_voltageConstants = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          DriveTrain.ksVolts,
          DriveTrain.kvVoltSecondsPerMeter,
          DriveTrain.kaVoltSecondsSquaredPerMeter),
          DriveTrain.kDriveKinematics,
      10));

    // Create config for trajectory
    m_config = new TrajectoryConfig(
      DriveTrain.kMaxSpeedMetersPerSecond,
      DriveTrain.kMaxAccelerationMetersPerSecondSquared,
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
 
  

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
  
  

     // Reset odometry to the starting pose of the trajectory.
     m_DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

     // Run path following command, then stop at the end.
     return ramseteCommand.andThen(() -> m_DriveSubsystem.tankDriveVolts(0, 0));
 
 }
 