package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Climber;

/* 
** Main class for handling Climber Substystem 
*/
public class ClimberSubsystem extends SubsystemBase {
  // Climber Motor
  private final CANSparkMax m_motorClimberLeft  = new CANSparkMax(Climber.kCANMotorClimberLeft, MotorType.kBrushed);
  private final CANSparkMax m_motorClimberRight = new CANSparkMax(Climber.kCANMotorClimberRight, MotorType.kBrushed);
  private final MotorControllerGroup m_motors   = new MotorControllerGroup(m_motorClimberLeft, m_motorClimberRight);
 
  // Class Constructor
  public ClimberSubsystem() {
    // Set motor inverted or not...
    m_motorClimberLeft.setInverted(false);
    m_motorClimberRight.setInverted(true);
  }  
      
  /**
   * Set Motors Power
   * @newPower new vslue to be applied [0.0 - 1.1]
   */
  public void setPower(double newPower){
    m_motors.set(newPower);
  }
}

 
