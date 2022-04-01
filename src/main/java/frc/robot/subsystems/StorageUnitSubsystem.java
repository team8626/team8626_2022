package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Simulation
  SendableChooser<Color> m_SIM_cargoChooser = new SendableChooser<>(); 
  private String m_name;

  // Color Sensing
  private final I2C.Port m_colorSensorPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private Color m_loadedColor = null;
  private boolean m_activated = false;

  // Class Constructor
  public StorageUnitSubsystem(String name, int CANID, I2C.Port I2CPort) {

    // Store Unit Name (Used for Dashboard and Simulation
    m_name = name;

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

  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putBoolean(m_name, this.isActive());
    SmartDashboard.putString("COLOR " + m_name, getColorAsString(m_loadedColor));

    if(RobotBase.isSimulation()){
      m_SIM_cargoChooser.addOption("BLUE", Cargo.kBlue);
      m_SIM_cargoChooser.addOption("RED", Cargo.kRed);
      m_SIM_cargoChooser.setDefaultOption("UNKNOWN", null);

      SmartDashboard.putData("SIM " + m_name, m_SIM_cargoChooser);
    }
  }

  // Update Dashboard
  public void updateDashboard(){
    if(RobotBase.isSimulation()){
      Color newColor = m_SIM_cargoChooser.getSelected();
            
      if (newColor != m_loadedColor) {m_loadedColor = newColor;}
    }
    SmartDashboard.putBoolean(m_name, this.isActive());
    SmartDashboard.putString("COLOR " + m_name, getColorAsString(m_loadedColor));
  };

  // Periodic Updates
  @Override
  public void periodic(){
    if(RobotBase.isReal()){
      // Update Loaded Color
      readLoadedColor();
    }
  }

  // Start conveying
  public void start(){
    m_motor.set(1.0);
    m_activated = true;
  }

  // Stop conveying
  public void stop(){
    m_motor.stopMotor();
    m_activated = false;

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
    
    if((m_loadedColor == Cargo.kBlue) || (m_loadedColor == Cargo.kRed)){
      ret_value = false;
    }
    return ret_value;
  }

  public boolean isActive(){
    return m_activated;
  }

  /** 
   * Set Motor Power. Used for manual control of the unit
   * @newPower new value to be applied [-1.0 ; 1.0]
   */ 
  public void setPower(double newPower){
      m_motor.set(newPower);
      if(newPower != 0){
        m_activated = true;
      } else {
        m_activated = false;
      }
  }

  // Get Color as String
  public String getColorAsString(Color color){
    String retval = "UNKNOWN";
    if(color == Cargo.kBlue){
      retval = "BLUE";
    }
    else if(color == Cargo.kRed){
      retval = "RED";
    }
    return retval;
  }    
}
