package frc.robot.subsystems;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Team8626 Libraries
import frc.robot.Constants.Storage;

/* 
** Main class for handling Storate Substystem 
*/
public class StorageSubsystem extends SubsystemBase {

  public enum Status {
    IDLE, LOADING, FORWARDING, DELIVERING, CANCELLED
  }

  // Storage Units
  private final StorageUnitSubsystem m_unitFront = new StorageUnitSubsystem("FRONT", Storage.kCANMotorStorageFront, Storage.kI2CColorSensorPortFront);
  private final StorageUnitSubsystem m_unitBack  = new StorageUnitSubsystem("BACK", Storage.kCANMotorStorageBack, Storage.kI2CColorSensorPortBack);

  // Internat States
  private Status m_status = Status.IDLE;

  // Class Constructor
  public StorageSubsystem() {
//    System.out.println("[STORAGE] Initialized");
  }  

  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putString("STORAGE_STATUS", statusToString(m_status));

  }

  // Update Dashboard (Called Periodically)
  public void updateDashboard(){
    SmartDashboard.putString("STORAGE_STATUS", statusToString(m_status));
  }

  // Periodic update of the states and functions
  @Override
  public void periodic(){
    
    // Update State Machine
    switch(m_status){
      case LOADING:
        // We are loading cargo 
        // Cargo detected in front unit, stop front unit
        if(!m_unitFront.isEmpty()){
          System.out.println("[STORAGE] Status LOADING - Stopping FRONT (isLoaded)");
          m_unitFront.stop();
          m_status = Status.IDLE;
        }
        break;
        
      case FORWARDING:
        // We are moving the cargo forward:
        //  Stop front unit when cargo is not detected anymore
        //  Stop back unit when cargo is detected 
        if(m_unitFront.isEmpty() && !m_unitBack.isEmpty()){
          System.out.println("[STORAGE] Status FORWARDING - Stopping FRONT (isEmpty) - DELAY");
          System.out.println("[STORAGE] Status FORWARDING - Stopping BACK (isLoaded)");
          m_unitFront.stop(0.5 /* seconds delay*/);
          m_unitBack.stop();
          m_status = Status.IDLE;
        }

        break;

      case DELIVERING:
        if(m_unitBack.isEmpty()){
          System.out.println("[STORAGE] Status DELIVERING - Stopping BACK (isEmpty) - DELAY");
          m_unitBack.stop(0.5 /* seconds delay*/);
          m_status = Status.IDLE;
        }
        break;
    
      case CANCELLED:
        // Last Command Cancelled, reset to IDLE
        m_status = Status.IDLE;
        break;
      case IDLE:
      default:
        // Do Nothing
    }
  }

  // Push all Cargos towards the Back...
  public void pushCargo(){
    if(m_status == Status.IDLE){
      // Back Unit is Empty
      if(m_unitBack.isEmpty()){
        if(m_unitFront.isEmpty()){
          // NOTHING TO DO HERE
        } else {
          // Push the Cargo "Forward"
          System.out.println("[STORAGE] Starting BOTH (Forwarding)");
          m_unitBack.start();
          m_unitFront.start();
          m_status = Status.FORWARDING;
        }
      }
      // Back Unit is Used...
      else {
        // NOTHING TO DO HERE
      }
    }
  }

  // Get ready to load Cargo (Start Frint Unit))
  public void loadCargo(){
    if(m_status == Status.IDLE){
      // Only if Front unit is empty and Status is IDLE
      if(m_unitFront.isEmpty()){
        System.out.println("[STORAGE] Starting FRONT (Loading)");
        m_unitFront.start();
        m_status = Status.LOADING;
      }
    }
  }

  // Deliver Cargo (Start Front Unit)
  public void deliverCargo(){
    if(m_status == Status.IDLE){
      // Only if Back unit is not empty
      if(!m_unitBack.isEmpty()){
        System.out.println("[STORAGE] Starting BACK (Deliver)");
        m_unitBack.start();
        m_status = Status.DELIVERING;
      }
    }
  }

  // Stop loading Cargo
  public void stopLoadingCargo(){
    if(m_status == Status.LOADING){
      System.out.println("[STORAGE] Status LOADING - Stopping FRONT (Cancelled)");
      m_unitFront.stop();
      m_status = Status.CANCELLED;
    }
  }
  
  // Is the storage full (2 Cargos loaded)
  public boolean isFull(){
    boolean ret_value = false;
    if ((!m_unitFront.isEmpty()) && (!m_unitBack.isEmpty())){
      ret_value = true;
    }
    return ret_value;
  }

  // Is the storage empty (no more cargo)
  public boolean isEmpty(){
    boolean ret_value = false;
    if (m_unitFront.isEmpty() && m_unitBack.isEmpty()){
      ret_value = true;
    }
    return ret_value;
  }  

  // Get Reference to the Front Unit
  public StorageUnitSubsystem getFrontUnit(){
    return m_unitFront;
  }

  // Get Reference to the Back Unit
  public StorageUnitSubsystem getBackUnit(){
    return m_unitBack;
  }    

  private String statusToString(Status newStatus){
    String retval = "UNKNOWN";
    switch(newStatus){
      case IDLE:
        retval = "IDLE";
        break;
      case LOADING:
        retval = "LOADING";
        break;
      case FORWARDING:
        retval = "FORWARDING";
        break;
      case DELIVERING:
        retval = "DELIVERING";
        break;
      default:
        retval = "UNKNOWN";
    }
    return retval;
  }

  public Status getStatus(){
    return m_status;
  }
}

