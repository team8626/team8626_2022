package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Shooter;

/* 
** Main class for handling Shooter Substystem 
*/
public class ShooterSubsystem extends SubsystemBase {
  // Intake Motor
  private final CANSparkMax m_motor = new CANSparkMax(Shooter.kCANMotorShooter, MotorType.kBrushed);

  // Internal States
  private double m_motorVoltage = 0.0;
  private boolean m_activated;

  // Class Constructor
  public ShooterSubsystem() {
    // Set motor inverted or not...
    m_motor.setInverted(false);

    // Initialize states
    m_motorVoltage = Shooter.kShooterVoltageLowGoal;
    this.deactivate();
  }  
      
  // Start Spinning
  public void activate(double voltage){
    m_motorVoltage = voltage;
    this.activate();
  }

  public void activate(){
    m_motor.setVoltage(m_motorVoltage);
    m_activated = true;
  }

  public void deactivate(){
    m_motor.stopMotor();
    m_activated = false;
  }

  public boolean isActive(){
    return m_activated;
  }
}
