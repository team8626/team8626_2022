// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// Team8626 Libraries
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.Constants.Shooter;

/**
 * Have the robot drive arcade style. 
 * */
public class LaunchCargoCommand extends SequentialCommandGroup {
  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new LaunchCargo command.
   * Since this doesn't receive target goal (high/Low), this command is going for default (low goal shooting)
   * 
   * @param shooter The shooter
   * @param storage The storage system to receive cargo from
   */
  public LaunchCargoCommand(StorageSubsystem storage, ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_storage = storage;
    
    m_shooter.setVoltage(Shooter.kShooterVoltageLowGoal);

    addCommands(
      new InstantCommand(m_shooter::activate, m_shooter),
      new WaitUntilCommand(Shooter.kShooterSpinSeconds),
      new DeliverCargoCommand(storage)
    );
    addRequirements(m_shooter, m_storage);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Called on finished or interrupted.
  @Override
  public void end(boolean interrupted) {
    new InstantCommand(m_shooter::deactivate, m_shooter);
  }
}
