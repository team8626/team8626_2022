// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Java Libraries

// WPI Library dependencies

// Team8626 Libraries
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Target;

/**
 * Start Shooting loaded Cargos.
 *      - Start the shooter
 *      - Deliver the Cargo to the shooter
 * 
 **/
public class StartDeliveringCommand extends CommandBase {
  // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem  m_shooter;
  private final StorageSubsystem m_storage;

  /**
   * Creates a new StartDeliveringCommand command.
   * 
   * @param storage The Intake
   * @param shooter The Storage
   */
  public StartDeliveringCommand(StorageSubsystem storage, ShooterSubsystem shooter) {
    m_storage = storage;
    m_shooter = shooter;
  }

  @Override
  public void initialize(){
    m_shooter.activate();
  }

  @Override
  public void execute(){
    // Adjust Speed Based on Cargo Color
    if(!m_storage.getBackUnit().isEmpty()){
      if(m_storage.getBackUnit().getCargoColor() != DriverStation.getAlliance()){
        System.out.println("[StartDeliveringCommand] DISCARD Next Cargo");
        m_shooter.setRPMTarget(Target.DISCARD);
      } else {
        m_shooter.setRPMTarget(m_shooter.getTarget());
      }

      // If Shooter is at speed request delivery from the storage.
      if(m_shooter.isAtSpeed()){
        m_storage.deliverCargo();
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean ret_value = false;
    if(m_storage.isEmpty()) {
      System.out.println("[StartDeliveringCommand] Storage is empty... Stopping");
      ret_value = true;
    }
    return ret_value;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // The Storage will stop by itself
    // Deactivate the Shooter (2 seconds delay)
    m_shooter.deactivate(2.0); 
  }
}
