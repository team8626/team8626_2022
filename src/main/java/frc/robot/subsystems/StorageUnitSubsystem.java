package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// Team8626 Libraries
import frc.robot.Constants.Cargo;

/* 
** Main class for handling Storage Unit Substystem 
*/
public class StorageUnitSubsystem extends SubsystemBase {
  // Storage Motor
  private final WPI_VictorSPX m_motor;

  // Color Sensing
  private final I2C.Port m_colorSensorPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private Color m_loadedColor = null;
  private String m_loadedColorString = "Not Initialized";

  // Class Constructor
  public StorageUnitSubsystem(int CANID, I2C.Port I2CPort) {

    // Set motor inverted or not...
    m_motor = new WPI_VictorSPX(CANID);
    m_motor.setInverted(true);

    // Set Color Sensor
    m_colorSensorPort = I2CPort;
    m_colorSensor = new ColorSensorV3(m_colorSensorPort);

    m_colorMatcher.addColorMatch(Cargo.kBlue);
    m_colorMatcher.addColorMatch(Cargo.kRed);

    readLoadedColor();
  }  

   // Periodic Updates
   @Override
   public void periodic(){
    // Update Loaded Color
    readLoadedColor();
  }

  // Start conveying
  public void start(){
    m_motor.set(1.0);
  }

  // Stop conveying
  public void stop(){
    m_motor.stopMotor();
  }

  // Return Current Loaded Color
  private void readLoadedColor(){
    //Run the color match algorithm on our detected color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
    m_loadedColor = match.color;
    
    if(m_loadedColor == Cargo.kBlue){
      m_loadedColorString = "BLUE";
    }
    else if(m_loadedColor == Cargo.kRed){
      m_loadedColorString = "RED";
    }
    else{
      m_loadedColorString = "EMPTY";
    }         
  }

  // Return Current Loaded Color
  public Color getLoadedColor(){
    return m_loadedColor;
  }

  // Return Current Loaded Color
  public boolean isEmpty(){
    boolean ret_value = true;
    
    if((m_loadedColor == Cargo.kBlue) || (m_loadedColor == Cargo.kRed)){
      ret_value = false;
    }
    return ret_value;
  }

  /** 
   * Set Motor Power. Used for manual control of the unit
   * @newPower new value to be applied [-1.0 ; 1.0]
   */ 
  public void setPower(double newPower){
      m_motor.set(newPower);
  }

  public String getColorAsString(){
    return m_loadedColorString;
  }
}
