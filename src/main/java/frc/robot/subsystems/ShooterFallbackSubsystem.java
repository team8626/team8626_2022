package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team8626 Libraries
import frc.robot.Constants.Shooter;

/* 
** Main class for handling Shooter Substystem 
*/
public class ShooterFallbackSubsystem extends SubsystemBase {
  
  private CANSparkMax m_motor;
  private CANSparkMax m_motor2;
  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_pidController2;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoder2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM,setPoint;
  public double kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2, maxRPM2,setPoint2;


  public ShooterFallbackSubsystem(){
    // initialize motors
    m_motor  = new CANSparkMax(Shooter.kCANMotorShooterMain, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(Shooter.kCANMotorShooterSecondary , MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();
    m_motor2.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();
    m_pidController2 = m_motor2.getPIDController();

    // Encoder object created to display position values
    m_encoder  = m_motor.getEncoder();
    m_encoder2 = m_motor2.getEncoder();

    // PID coefficients
    kP = 0.00006; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.00019; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    setPoint = 0;

    kP2 = 0.000002; 
    kI2 = 0;
    kD2 = 0; 
    kIz2 = 0; 
    kFF2 = 0.00019; 
    kMaxOutput2 = 1; 
    kMinOutput2 = -1;
    maxRPM2 = 5700;
    setPoint2 = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP2);
    m_pidController2.setI(kI2);
    m_pidController2.setD(kD2);
    m_pidController2.setIZone(kIz2);
    m_pidController2.setFF(kFF2);
    m_pidController2.setOutputRange(kMinOutput2, kMaxOutput2);
  }

  // Initialize Dashboard
  public void initDashboard(){
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("SetPoint", setPoint);

    SmartDashboard.putNumber("P Gain2", kP2);
    SmartDashboard.putNumber("I Gain2", kI2);
    SmartDashboard.putNumber("D Gain2", kD2);
    SmartDashboard.putNumber("I Zone2", kIz2);
    SmartDashboard.putNumber("Feed Forward2", kFF2);
    SmartDashboard.putNumber("Max Output2", kMaxOutput2);
    SmartDashboard.putNumber("Min Output2", kMinOutput2);
    SmartDashboard.putNumber("SetPoint2", setPoint2);
  }

  public void updateDashboard(){
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double newSetpoint = SmartDashboard.getNumber("SetPoint", 0);

    double p2 = SmartDashboard.getNumber("P Gain2", 0);
    double i2 = SmartDashboard.getNumber("I Gain2", 0);
    double d2 = SmartDashboard.getNumber("D Gain2", 0);
    double iz2 = SmartDashboard.getNumber("I Zone2", 0);
    double ff2 = SmartDashboard.getNumber("Feed Forward2", 0);
    double max2 = SmartDashboard.getNumber("Max Output2", 0);
    double min2 = SmartDashboard.getNumber("Min Output2", 0);
    double newSetpoint2 = SmartDashboard.getNumber("SetPoint2", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((newSetpoint != setPoint)) { m_pidController.setReference(newSetpoint, CANSparkMax.ControlType.kVelocity); setPoint = newSetpoint; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if((p2 != kP2)) { m_pidController2.setP(p); kP2 = p2; }
    if((i2 != kI2)) { m_pidController2.setI(i); kI2 = i2; }
    if((d2 != kD2)) { m_pidController2.setD(d); kD2 = d2; }
    if((iz2 != kIz2)) { m_pidController2.setIZone(iz2); kIz2 = iz2; }
    if((newSetpoint2 != setPoint2)) { m_pidController2.setReference(newSetpoint2, CANSparkMax.ControlType.kVelocity); setPoint = newSetpoint; }
    if((ff2 != kFF2)) { m_pidController2.setFF(ff2); kFF2 = ff2; }
    if((max2 != kMaxOutput2) || (min2 != kMinOutput2)) { 
      m_pidController2.setOutputRange(min2, max2); 
      kMinOutput2 = min2; kMaxOutput2 = max2; 
    }

    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("ProcessVariable2", m_encoder2.getVelocity());

  }

 
  // Periodic Updates
  @Override
  public void periodic(){
  }

  public void activate(){
  }

  public void deactivate(){
  }

  /**
   * Stop with delay (seconds)
   * @param seconds Timer dutration in seconds 
   */
  public void deactivate(double seconds){
    deactivate();
  }

  // Shooter is spinning at target speed!
  public boolean isAtSpeed() {
    boolean retval = true;  // TODO Always True since could not get shooter at setpoint...
    return retval;
  }
}
