package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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
  private final DoubleSolenoid m_cylinderLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.kcylinderLeftExtend, Intake.kcylinderLeftRetract);
  private final DoubleSolenoid m_cylinderRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.kcylinderRightExtend, Intake.kcylinderRightRetract);
  
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
      
  /**
   * Deploy the intake.
   * Pushing the intake assembly out using the 2 pneumatic cylinders
   * then start spinning the intake.
   */
  public void activate(){
    if (m_activated == false) {
      // Push the assembly out
      this.deploy();

      // Start Spinning the intake
      this.startSpinning();

      // Set Action to Finished
      m_activated = true;
    }
  }

  /**
   * Retract the intake.
   * Pushing the intake assembly out using the 2 pneumatic cylinders
   * then start spinning the intake.
   */
  public void deactivate(){
    if (m_activated == true) {
      // Stop Spinning the intake
      this.stopSpinning();

      // Retract the assembly
      this.retract();

      // Set Action to Finished
      m_activated = false;
    }
  }

  // Push the Assembly Out
  private void deploy(){
    // Extend Cylinders
    new PrintCommand("[INTAKE] Intake Deployed");
    m_cylinderLeft.set(Value.kForward);
    m_cylinderRight.set(Value.kForward);

    // Stop pushing on cylinders (keep the assembly "loose")
    m_cylinderLeft.set(Value.kOff);
    m_cylinderRight.set(Value.kOff);
  }
  
  // Pull The Assembly in
  private void retract(){
    // Retract Cylinders
    new PrintCommand("[INTAKE] Intake Stowed");
    m_cylinderLeft.set(Value.kReverse);
    m_cylinderRight.set(Value.kReverse);
  }
  
  // Start Rotation of the Intake
  private void startSpinning(){
    new PrintCommand("[INTAKE] Spinning");
    m_motorIntake.set(m_intakePower);
  }

  // Stop Rotation of the Intake
  private void stopSpinning(){
    new PrintCommand("[INTAKE] STOP Spinning");
    m_motorIntake.stopMotor();
  }

  @Override
  public void periodic() {
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
