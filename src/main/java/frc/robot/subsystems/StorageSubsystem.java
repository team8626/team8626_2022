package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  }

}
