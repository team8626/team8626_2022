package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.Climber;

/* 
** Main class for handling Climber Substystem 
*/
public class ClimberSubsystem extends SubsystemBase {
  // Climber Motor
  private final WPI_VictorSPX m_motor  = new WPI_VictorSPX(Climber.kCANMotorClimber);

  private boolean m_enabled = true;

  // Class Constructor
  public ClimberSubsystem() {
    // Set motor inverted or not...
    m_motor.setInverted(false);
  }  
      
  /**
   * Set Motors Power
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  public void setPower(double newPower){
    if(m_enabled){
      m_motor.set(newPower);
    }
  }

  // Activate The Climber
  public void setEnabled(){
    m_enabled = true;
  }

  // Deactivate The Climber
  public void setDisabled(){
    m_enabled = false;
  }
}

 
