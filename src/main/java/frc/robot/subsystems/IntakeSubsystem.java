package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.Intake;

/* 
** Main class for handling Intake Substystem 
*/
public class IntakeSubsystem extends SubsystemBase {
  // Intake Motor
  private final WPI_VictorSPX m_motorIntake = new WPI_VictorSPX(Intake.kCANMotorIntake);
  private final double m_intakePower = Intake.kMotorIntakePower;

  // Pneumatics
  //private final Compressor m_compressor = new Compressor(Intake.kPneumaticModuleID, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid m_Cylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.kCylinderExtend, Intake.kCylinderRetract);
  
  // Internal States
  private boolean m_activated;

  // Class Constructor
  public IntakeSubsystem() {

    // Set motor inverted or not...
    m_motorIntake.setInverted(false);

    // Initializa states
    this.stopSpinning();
    this.retract();
    m_activated = false;
  }  
  
  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putBoolean("INTAKE", m_activated);
  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
    SmartDashboard.putBoolean("INTAKE", m_activated);
  }

  /**
   * Deploy the intake.
   * Pushing the intake assembly out using the 2 pneumatic cylinders
   * then start spinning the intake.
   */
  public void activate(){
    // if (m_activated == false) {
      // Push the assembly out
      this.deploy();

      // Start Spinning the intake
      this.startSpinning();

      // Set Action to Finished
      m_activated = true;
      System.out.println("[INTAKE] Activated");

    // }
  }

  /**
   * Retract the intake.
   * Pushing the intake assembly out using the 2 pneumatic cylinders
   * then start spinning the intake.
   */
  public void deactivate(){
    // if (m_activated == true) {
      // Stop Spinning the intake
      this.stopSpinning();

      // Retract the assembly
      this.retract();

      // Set Action to Finished
      m_activated = false;
      System.out.println("[INTAKE] Deactivated");
    // }
  }

  // Set assembly passive
  private void passive(){
    // Extend Cylinders
    m_Cylinder.set(Value.kOff);
  }
  
  // Push the Assembly Out
  private void deploy(){
    // Extend Cylinders
    m_Cylinder.set(Value.kForward);
  }
  
  // Pull The Assembly in
  private void retract(){
    // Retract Cylinders
    m_Cylinder.set(Value.kReverse);
  }
  
  // Start Rotation of the Intake
  private void startSpinning(){
    m_motorIntake.set(m_intakePower);
  }

  // Stop Rotation of the Intake
  private void stopSpinning(){
    m_motorIntake.stopMotor();
  }

  @Override
  public void periodic() {}

  public boolean isActive(){
    return m_activated;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    //m_drive.setMaxOutput(maxOutput);
  }
}
