package frc.robot.subsystems;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// Team8626 Libraries
import frc.robot.commands.LoadStorageUnitCommand;
import frc.robot.commands.UnloadStorageUnitCommand;
import frc.robot.Constants.Cargo;
import frc.robot.Constants.Storage;

/* 
** Main class for handling Storate Substystem 
*/
public class StorageSubsystem extends SubsystemBase {

  // Storage Units
  private final StorageUnitSubsystem m_unitFront = new StorageUnitSubsystem("FRONT", Storage.kCANMotorStorageFront, Storage.kI2CColorSensorPortFront);
  private final StorageUnitSubsystem m_unitBack  = new StorageUnitSubsystem("BACK", Storage.kCANMotorStorageBack, Storage.kI2CColorSensorPortBack);

  // Remember Colors
  private Color m_loadedColorFront = null;
  private Color m_loadedColorBack  = null;

  // Class Constructor
  public StorageSubsystem() {}  

  // Initialize Dashboard
  public void initDashboard(){
  }

  // Update Dashboard (Called Periodically)
  public void updateDashboard(){
  }

  // Push all Cargos towards the Back...
  public void storeForward(){
      // Back Unit is Empty
      if(m_unitBack.isEmpty()){
        if(m_unitFront.isEmpty()){
          // NOTHING TO DO HERE
        } else {
          // Push the Cargo "Forward"
          new ParallelCommandGroup(
            new LoadStorageUnitCommand(m_unitBack),
            new UnloadStorageUnitCommand(m_unitFront).withTimeout(Storage.kTimeoutStorageUnit)
          );
        }
    } 
    // Back Unit is Used...
    else {
      // NOTHING TO DO HERE
    }
  }

  // Is the storage full (2 Cargos loaded)
  public boolean isFull(){
    boolean ret_value = false;
    if (!m_unitFront.isEmpty() && !m_unitBack.isEmpty()){
      ret_value = true;
    }
    return ret_value;
  }

  // Is the storage empty (no more cargo)
  public boolean isEmpty(){
    boolean ret_value = false;
    if (!m_unitFront.isEmpty() || !m_unitBack.isEmpty()){
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
}

