package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// Team8626 Libraries
import frc.robot.Constants.Shooter;

/* 
** Main class for handling Shooter Substystem 
*/
public class ShooterSubsystem extends SubsystemBase {
  // Intake Motor
  private final WPI_VictorSPX m_motor = new WPI_VictorSPX(Shooter.kCANMotorShooter);

  // Internal States
  private double m_motorVoltage = 0.0;
  private boolean m_activated;

  // Class Constructor
  public ShooterSubsystem() {
    // Set motor inverted or not...
    m_motor.setInverted(false);

    // Initializa states
    this.setVoltage(Shooter.kShooterVoltageLowGoal);
    this.deactivate();
  }  
      
  // Start Spinning
  public void activate(double voltage){
    m_motor.setVoltage(voltage);
    this.activate();
    new PrintCommand("[SHOOTER] Spinning");
  }

  public void activate(){
    m_motor.setVoltage(m_motorVoltage);
    m_activated = true;
  }

  public void deactivate(){
    m_motor.stopMotor();
    m_activated = false;
    new PrintCommand("[SHOOTER] STOP Spinning");
  }

  public void speedUp(){
    m_motorVoltage+= 0.1;
    m_motor.setVoltage(m_motorVoltage);
  }

  public void speedDown(){
    m_motorVoltage-= 0.1;
    m_motor.setVoltage(m_motorVoltage);
  }

  public boolean isActive(){
    return m_activated;
  }

  public void setVoltage(double voltage){
    m_motorVoltage = voltage;
  }
}
