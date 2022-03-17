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
public class TestTrajectory extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private final TrajectoryConfig m_config;
  private final DifferentialDriveVoltageConstraint m_voltageConstants;
  // private final Trajectory m_trajectory;
  // private final TrajectoryConfig m_trajectoryConfig;

  /**
   * Creates a new ArcadeDrive command.
   * @param drivetrain The drivetrain subsystem to drive
   */
  public TestTrajectory(DriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;

    // Create Test Trajectory
    // m_trajectory = new...
  // Create a voltage constraint to ensure we don't accelerate too fast
  m_voltageConstants = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
        DriveTrain.ksVolts,
        DriveTrain.kvVoltSecondsPerMeter,
        DriveTrain.kaVoltSecondsSquaredPerMeter),
        DriveTrain.kDriveKinematics,
    10);

  // Create config for trajectory
  m_config = new TrajectoryConfig(
    DriveTrain.kMaxSpeedMetersPerSecond,
    DriveTrain.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveTrain.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(m_voltageConstants);

      Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through the se two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          m_config);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RamseteCommand testRamseteCommand =
    new RamseteCommand(
      testTrajectory,
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
    // TODO: create ramseteController for the trajectory
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if( 0/*TODO: Find condition Finished trajectory*/ ) {
      return true;
    }
    return false;
  }
  
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
