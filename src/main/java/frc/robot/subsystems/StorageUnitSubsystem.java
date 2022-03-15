package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.Cargo;

/* 
** Main class for handling Storage Unit Substystem 
*/
public class StorageUnitSubsystem extends SubsystemBase {
  // Storage Motor
  private final CANSparkMax m_motor;

  // Color Sensing
  private final I2C.Port m_colorSensorPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private Color m_loadedColor = null;

  // Class Constructor
  public StorageUnitSubsystem(int CANID, I2C.Port I2CPort) {

    // Set motor inverted or not...
    m_motor = new CANSparkMax(CANID, MotorType.kBrushed);
    m_motor.setInverted(false);

    // Set Color Sensor
    m_colorSensorPort = I2CPort;
    m_colorSensor = new ColorSensorV3(m_colorSensorPort);

    m_colorMatcher.addColorMatch(Cargo.kBlue);
    m_colorMatcher.addColorMatch(Cargo.kRed);

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
  }

  // Return Current Loaded Color
  public Color getLoadedColor(){
    return m_loadedColor;
  }

  // Return Current Loaded Color
  public boolean isEmpty(){
    boolean ret_value = true;
    
    if(m_loadedColor == null){
      ret_value = false;
    }
    return ret_value;
  }

  @Override
  public void periodic() {
  }

}
