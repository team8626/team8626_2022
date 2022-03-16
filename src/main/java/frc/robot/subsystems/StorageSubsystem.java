package frc.robot.subsystems;

// Java Libraries
// import java.util.function.DoubleSupplier;

// WPI Library dependencies
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;

// Team8626 Libraries
import frc.robot.commands.LoadStorageUnit;
import frc.robot.commands.UnloadStorageUnit;
import frc.robot.Constants.Storage;

/* 
** Main class for handling Storate Substystem 
*/
public class StorageSubsystem extends SubsystemBase {
  // Storage Units
  private final StorageUnitSubsystem m_unitFront = new StorageUnitSubsystem(Storage.kCANMotorStorageFront, Storage.kI2CColorSensorPortFront);
  private final StorageUnitSubsystem m_unitBack  = new StorageUnitSubsystem(Storage.kCANMotorStorageBack, Storage.kI2CColorSensorPortBack);

  // Class Constructor
  public StorageSubsystem() {
  }  

  @Override
  public void periodic() {
    // Regularly Check if Cargos need to be forwarded into storage.
    // TODO: move this to RobotContainer Default Command
    this.storeForward();
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
            new PrintCommand("[STORAGE] Moving Cargo Forward"),
            new LoadStorageUnit(m_unitBack),
            new UnloadStorageUnit(m_unitFront)
          );
        }
    } 
    // Back Unit is Used...
    else {
      // NOTHING TO DO HERE
    }
  }

  // Is the storage full (2 Cargios loaded)
  public boolean isFull(){
    boolean ret_value = false;
    if (!m_unitFront.isEmpty() && !m_unitBack.isEmpty()){
      ret_value = true;
    }
    return ret_value;
  }

  // Start Loading
  public void load(){
    new PrintCommand("[STORAGE] Ready to Receive Cargo");
    // TODO: IMPLEMENT THIS FOR LOADING!!!
  }

  // Stop Loading
  public void abort(){
    new PrintCommand("[STORAGE] NOT Ready to Receive Cargo");
    // TODO: IMPLEMENT THIS FOR STOPPING LOADING!!!
  }


}
