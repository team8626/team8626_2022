package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.Climber;

/* 
** Main class for handling Climber Substystem 
*/
public class ClimberSubsystem extends SubsystemBase {
  // Climber Motor
  private final WPI_VictorSPX m_motorLeft  = new WPI_VictorSPX(Climber.kCANMotorClimberLeft);
  private final WPI_VictorSPX m_motorRight  = new WPI_VictorSPX(Climber.kCANMotorClimberRight);

  private boolean m_enabled = true;
  private boolean m_activated = false;
  
  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putBoolean("CLIMBER", m_activated);
  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
    SmartDashboard.putBoolean("CLIMBER", m_activated);
  }
  
  // Class Constructor
  public ClimberSubsystem() {
    // Set motor inverted or not...
    m_motorLeft.setInverted(false);
    m_motorRight.setInverted(true);
  }  
      
  /**
   * Set Motors Power
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  public void setPower(double newPower){
    if(m_enabled){
      if(newPower != 0){
        m_activated = true;
      }
      else{
        m_activated = false;
      }
      m_motorLeft.set(newPower);
      m_motorRight.set(newPower);
    }
  }

  // // Activate The Climber
  // public void setEnabled(){
  //   m_enabled = true;
  // }

  // // Deactivate The Climber
  // public void setDisabled(){
  //   m_enabled = false;
  // }
}

 
