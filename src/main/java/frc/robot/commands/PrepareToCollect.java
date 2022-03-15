// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.CommandBase;

// Team8626 Libraries
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Have the robot drive arcade style. 
 * */
public class PrepareToCollect extends CommandBase {
  private final IntakeSubsystem m_intake;

  /**
   * Creates a new PrepareToShoot command.
   * Since this doesn't receive target goal, tyhis command is going for default (low goal shooting)
   * 
   * @param shooter The shooter
   */
  public PrepareToCollect(IntakeSubsystem intake) {
    m_intake = intake;

    addRequirements(m_intake);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_intake.activate();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // TODO: What is the finished condition
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_intake.deactivate();
  }
}
