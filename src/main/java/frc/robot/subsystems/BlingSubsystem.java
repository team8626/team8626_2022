package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Bling;

/* 
** Main class for handling Bling Substystem (LEDs)
*/
public class  BlingSubsystem extends SubsystemBase {
  // Local Members
  private final AddressableLED m_led = new AddressableLED(Bling.kPWMPort1);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Bling.kBufferSize);

  // States for LEDs
  enum State{SOLID, OFF, FADING};
  State m_state;

  // Class Constructor
  public BlingSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }  

  // Periodic Updates
  @Override
  public void periodic(){
    switch(m_state){
      case FADING:
        break;
      case OFF:
      case SOLID:
      default:
        break;
    }
  }

  /**
   * setRGB Sets the LED Strip to one single solid RGB Color
   * @param r
   * @param g
   * @param b
   */
  public void setRGB(int r, int g, int b){
    // Duplicate Value afer Half of the buffer
    int bufferSize = m_ledBuffer.getLength();
    int halfBufferSize = bufferSize / 2;
    for (var i = 0; i < halfBufferSize; i++) {
      m_ledBuffer.setRGB(i, r, g, b);
      m_ledBuffer.setRGB(halfBufferSize + i, r, g, b);
    }
    m_state = State.SOLID;
    m_led.setData(m_ledBuffer);
  }

  /**
   * Set Color Based on predefined Colors in {@link Color}
   **/
  public void setColor(Color color){
    int r = (int)(color.red   * 255);
    int g = (int)(color.green * 255);
    int b = (int)(color.blue  * 255);
    setRGB(r, g, b);
  }

  public void setAllianceColor(){
    if(DriverStation.getAlliance() == Alliance.Blue){
      setColor(Color.kFirstBlue);
    } else if(DriverStation.getAlliance() == Alliance.Red){
      setColor(Color.kFirstRed);
    } else {
      setColor(Color.kLightPink);
    }
  }

}
