package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.Climber;

/* 
** Main class for handling Climber Substystem 
*/
public class ClimberSubsystem extends SubsystemBase {
  // Climber Motor
  private final WPI_VictorSPX m_motorClimberLeft  = new WPI_VictorSPX(Climber.kCANMotorClimberLeft);
  private final WPI_VictorSPX m_motorClimberRight = new WPI_VictorSPX(Climber.kCANMotorClimberRight);
  private final MotorControllerGroup m_motors   = new MotorControllerGroup(m_motorClimberLeft, m_motorClimberRight);
 
  // Class Constructor
  public ClimberSubsystem() {
    // Set motor inverted or not...
    m_motorClimberLeft.setInverted(false);
    m_motorClimberRight.setInverted(true);
  }  
      
  /**
   * Set Motors Power
   * @newPower new value to be applied [0.0 - 1.1]
   */
  // TODO Link That to commands/buttons
  public void setPower(double newPower){
    m_motors.set(newPower);
  }
}

 
