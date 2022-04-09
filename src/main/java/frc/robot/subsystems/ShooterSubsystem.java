package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team8626 Libraries
import frc.robot.Constants.Shooter;

/* 
** Main class for handling Shooter Substystem 
*/


public class ShooterSubsystem extends SubsystemBase {

private CANSparkMax m_motor;
private CANSparkMax m_motor3;
private SparkMaxPIDController m_pidController;
private SparkMaxPIDController m_pidController3;
private RelativeEncoder m_encoder;
private RelativeEncoder m_encoder3;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM,setPoint;
public double kP3, kI3, kD3, kIz3, kFF3, kMaxOutput3, kMinOutput3, maxRPM3,setPoint3;
  // Shooter Motors
  // private final CANSparkMax m_motor1 = new CANSparkMax(Shooter.kCANMotorShooterMain, MotorType.kBrushless);
  // private final CANSparkMax m_motor2 = new CANSparkMax(Shooter.kCANMotorShooterSecondary, MotorType.kBrushless);

  // // Encoders
  // private final RelativeEncoder m_encoder1       = m_motor1.getEncoder();
  // private final RelativeEncoder m_encoder2  = m_motor2.getEncoder();

  // // PID Controllers
  // private final SparkMaxPIDController m_pidController1 = m_motor1.getPIDController();
  // private final SparkMaxPIDController m_pidController2 = m_motor2.getPIDController();

  // private double m_p1  = Shooter.kP_Main;
  // private double m_i1  = Shooter.kI_Main;
  // private double m_d1  = Shooter.kD_Main;
  // private double m_iz1 = Shooter.kIz_Main;
  // private double m_ff1 = Shooter.kFF_Main;

  // private double m_p2  = Shooter.kP_Secondary;
  // private double m_i2  = Shooter.kI_Secondary;
  // private double m_d2  = Shooter.kD_Secondary;
  // private double m_iz2 = Shooter.kIz_Secondary;
  // private double m_ff2 = Shooter.kFF_Secondary;

  // private double m_minOutput = Shooter.kMinOutput;
  // private double m_maxOutput = Shooter.kMaxOutput;
 

  // Store Velocity set by User on Dashboard
  // private double m_RPMInput1 = Shooter.kShooterMainRPM_LowGoal;
  // private double m_RPMInput2 = Shooter.kShooterSecondaryRPM_LowGoal;
  private double m_RPMInput1 = Shooter.kShooterMainRPM_HighGoal;
  private double m_RPMInput2 = Shooter.kShooterSecondaryRPM_HighGoal;
  private double m_RPMLastCustom1 = m_RPMInput1;
  private double m_RPMLastCustom2 = m_RPMInput2;
  
  // Store SetPoint
  private double m_RPMSetPoint1 = 0;
  private double m_RPMSetPoint2 = 0;
  private double m_RPMLastActiveSetPoint1 = m_RPMInput1;
  private double m_RPMLastActiveSetPoint2 = m_RPMInput2;

  // Shooter Targets
  public enum Target {LOW, HIGH, DISCARD, USER};

  // Internal Stats
  private boolean m_activated;

  // private SendableChooser<Targe> m_targetChooser = new SendableChooser<>();
  // private Target m_shooterTarget = Shooter.kDefaultTarget;

  private Timer m_delayedStopTimer = new Timer();
  private double m_delayedStopDuration = 1.0;
  private boolean m_delayedStopStarted = false;
  
  // Class Constructor
  public ShooterSubsystem() {

    // // Set motor inverted or not...
    // m_motor1.setInverted(true);
    // m_motor2.setInverted(false);

    // // Set PID coefficients
    // m_motor1.restoreFactoryDefaults();
    // m_motor1.setIdleMode(IdleMode.kBrake);
    // m_pidController1.setP(m_p1);
    // m_pidController1.setI(m_i1);
    // m_pidController1.setD(m_d1);
    // m_pidController1.setIZone(m_iz1);
    // m_pidController1.setFF(m_ff1);
    // m_pidController1.setOutputRange(m_minOutput, m_maxOutput);

    // m_motor2.restoreFactoryDefaults();
    // m_motor2.setIdleMode(IdleMode.kBrake);
    // m_pidController2.setP(m_p2);
    // m_pidController2.setI(m_i2);
    // m_pidController2.setD(m_d2);
    // m_pidController2.setIZone(m_iz2);
    // m_pidController2.setFF(m_ff2);
    // m_pidController2.setOutputRange(m_minOutput, m_maxOutput);
    m_motor  = new CANSparkMax(Shooter.kCANMotorShooterMain, MotorType.kBrushless);
    m_motor3 = new CANSparkMax(Shooter.kCANMotorShooterSecondary , MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();
    m_motor3.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();
    m_pidController3 = m_motor3.getPIDController();

    // Encoder object created to display position values
    m_encoder  = m_motor.getEncoder();
    m_encoder3 = m_motor3.getEncoder();

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

    kP3 = 0.000002; 
    kI3 = 0;
    kD3 = 0; 
    kIz3 = 0; 
    kFF3 = 0.00019; 
    kMaxOutput3 = 1; 
    kMinOutput3 = -1;
    maxRPM3 = 5700;
    setPoint3 = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController3.setP(kP3);
    m_pidController3.setI(kI3);
    m_pidController3.setD(kD3);
    m_pidController3.setIZone(kIz3);
    m_pidController3.setFF(kFF3);
    m_pidController3.setOutputRange(kMinOutput3, kMaxOutput3);
  }  

  // Initialize Dashboard
  public void initDashboard(){
    // Display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("MAIN P Gain", m_p1);
    // SmartDashboard.putNumber("MAIN I Gain", m_i1);
    // SmartDashboard.putNumber("MAIN D Gain", m_d1);
    // SmartDashboard.putNumber("MAIN I Zone", m_iz1);
    // SmartDashboard.putNumber("MAIN Feed Forward", m_ff1);

    // SmartDashboard.putNumber("SECONDARY P Gain", m_p2);
    // SmartDashboard.putNumber("SECONDARY I Gain", m_i2);
    // SmartDashboard.putNumber("SECONDARY D Gain", m_d2);
    // SmartDashboard.putNumber("SECONDARY I Zone", m_iz2);
    // SmartDashboard.putNumber("SECONDARY Feed Forward", m_ff2);

    // SmartDashboard.putNumber("Max Output", m_minOutput);
    // SmartDashboard.putNumber("Min Output", m_maxOutput);

    // Display Wheel RPM on Dashboard
    SmartDashboard.putNumber("Input Wheel RPM", m_RPMInput1);
    SmartDashboard.putNumber("Input Back Spin RPM", m_RPMInput2);

    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPoint1);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPoint2);

    SmartDashboard.putNumber("Actual Wheel RPM", m_encoder.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoder3.getVelocity());

    SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Taget Selection
    // m_targetChooser.addOption(targetToString(Target.USER), Target.USER);
    // m_targetChooser.addOption(targetToString(Target.LOW), Target.LOW);
    // m_targetChooser.addOption(targetToString(Target.HIGH), Target.HIGH);
    // m_targetChooser.addOption(targetToString(Target.DISCARD), Target.DISCARD);
    // m_targetChooser.setDefaultOption(targetToString(m_shooterTarget), m_shooterTarget);
    // SmartDashboard.putData("SHOOTER Target", m_targetChooser);
  }

  // Update Dashboard (Called Periodically)
  public void updateDashboard(){
    // Read SmartDashboard PID Coefficients
    // double p1 = SmartDashboard.getNumber("MAIN P Gain", 0);
    // double i1 = SmartDashboard.getNumber("MAIN I Gain", 0);
    // double d1 = SmartDashboard.getNumber("MAIN D Gain", 0);
    // double iz1 = SmartDashboard.getNumber("MAIN I Zone", 0);
    // double ff1  = SmartDashboard.getNumber("MAIN Feed Forward", 0);

    // double pSec = SmartDashboard.getNumber("SECONDARY P Gain", 0);
    // double iSec = SmartDashboard.getNumber("SECONDARY I Gain", 0);
    // double dSec = SmartDashboard.getNumber("SECONDARY D Gain", 0);
    // double izSec = SmartDashboard.getNumber("SECONDARY I Zone", 0);
    // double ffSec = SmartDashboard.getNumber("SECONDARY Feed Forward", 0);

    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    double input_main      = SmartDashboard.getNumber("Input Wheel RPM", 0);
    double input_secondary = SmartDashboard.getNumber("Input Back Spin RPM", 0);

    if((input_main != m_RPMInput1)) {m_RPMInput1 = input_main;}
    if((input_secondary != m_RPMInput2)) {m_RPMInput2 = input_secondary;}

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p1 != m_p))   { m_pidController1.setP(p1); m_p1 = p1; }
    // if((i1 != m_i))   { m_pidController1.setI(i1); m_i1 = i1; }
    // if((d1 != m_d1))   { m_pidController1.setD(d1); m_d1 = d1; }
    // if((iz1 != m_iz1)) { m_pidController1.setIZone(iz1); m_iz1 = iz1; }
    // if((ff1 != m_ff1)) { m_pidController1.setFF(ff1); m_ff1 = ff1; }

    // if((pSec != m_p2))   { m_pidController2.setP(pSec); m_p2 = pSec; }
    // if((iSec != m_i2))   { m_pidController2.setI(iSec); m_i2 = iSec; }
    // if((dSec != m_d2))   { m_pidController2.setD(dSec); m_d2 = dSec; }
    // if((izSec != m_iz2)) { m_pidController2.setIZone(izSec); m_iz2 = izSec; }
    // if((ffSec != m_ff2)) { m_pidController2.setFF(ffSec); m_ff2 = ffSec; }

    // if((max != m_maxOutput) || (min != m_minOutput)) { 
    //   m_pidController1.setOutputRange(min, max); 
    //   m_pidController2.setOutputRange(min, max); 
    //   m_minOutput = min; 
    //   m_maxOutput = max; 
    // }

    // Target target = m_targetChooser.getSelected();
    // if(target == Target.USER){
    //   if(m_shooterTarget != Target.USER){
    //     // Enterring in Custom mode
    //     // Resel last Custom Values
    //     // m_RPMInput1      = 99999; // Dummy value to force update
    //     // m_RPMInput2 = 99999; // Dummy value to force update
    //     input_main          = m_RPMLastCustom1;
    //     input_secondary     = m_RPMLastCustom2;

    //     m_shooterTarget = Target.USER; 
    //     m_targetChooser.setDefaultOption(targetToString(m_shooterTarget), m_shooterTarget);
    //     SmartDashboard.putData("SHOOTER Target", m_targetChooser);
    //   }
    //   // Read User input for RPM and store them
    //   if((input_main != m_RPMInput1)) 
    //   {
    //     m_RPMInput1 = input_main;
    //     if(m_activated){
    //       m_RPMSetPoint1 = m_RPMInput1;
    //     } else {
    //       m_RPMLastActiveSetPoint1 = m_RPMInput1;
    //     }
    //     m_RPMLastCustom1 = m_RPMInput1;
    //     System.out.println("[SHOOTER] New Main RPM: " + m_RPMInput1);
    //   }
      
    //   if((input_secondary != m_RPMInput2))
    //   { 
    //     m_RPMInput2 = input_secondary;
    //     if(m_activated){
    //       m_RPMSetPoint2 = m_RPMInput2;
    //     } else {
    //       m_RPMLastActiveSetPoint2 = m_RPMInput2;
    //     }
    //     m_RPMLastCustom2 = m_RPMInput2;

    //     System.out.println("[SHOOTER] New Back Spin RPM: " + m_RPMInput2);
    //   } 
    // } else {
    //   // Predefined Values (LOW, HIGH, DISCARD)
    //   if(target != m_shooterTarget){
    //     m_shooterTarget = target;
    //     setRPMTarget(m_shooterTarget);

    //     // Adjust Input values for display on shuffleboard
    //     if(m_activated){
          // m_RPMInput1 = m_RPMSetPoint1;
          // m_RPMInput2 = m_RPMSetPoint2;
    //     } else {
    //       m_RPMInput1 = m_RPMLastActiveSetPoint1;
    //       m_RPMInput2 = m_RPMLastActiveSetPoint2;
    //     }
    //   }
    // }
    if(m_activated){
      m_RPMSetPoint1  = m_RPMInput1;
      m_RPMSetPoint2 = m_RPMInput2;
    } else {
      m_RPMInput1 = m_RPMLastActiveSetPoint1;
      m_RPMInput2 = m_RPMLastActiveSetPoint2;
    }


    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Update Speed values ( User, SetPoint, Actual)
    SmartDashboard.putNumber("Input Wheel RPM", m_RPMInput1);
    SmartDashboard.putNumber("Input Back Spin RPM", m_RPMInput2);

    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPoint1);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPoint2);

    SmartDashboard.putNumber("Actual Wheel RPM", m_encoder.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoder3.getVelocity());
    
  //   SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

  }

  // Periodic Updates
  @Override
  public void periodic(){
    // Keep Adjusting RPM Setpoint
    m_pidController.setReference(m_RPMSetPoint1, CANSparkMax.ControlType.kVelocity);
    m_pidController3.setReference(m_RPMSetPoint2, CANSparkMax.ControlType.kVelocity);

    // Check the delayed Stop timer
    if(m_delayedStopStarted && (m_delayedStopTimer.hasElapsed(m_delayedStopDuration))){
      m_delayedStopStarted = false;
      m_delayedStopTimer.stop();
      this.deactivate();
    }
  }

  public void activate(){
    m_RPMSetPoint1      = m_RPMLastActiveSetPoint1;
    m_RPMSetPoint2 = m_RPMLastActiveSetPoint2;
    
    m_activated = true;
    System.out.println("[SHOOTER] Activated (" + m_RPMSetPoint1 + ", " + m_RPMSetPoint2 +")");

  }

  public void deactivate(){
     m_activated = false;
    
    // Set new setpoint to 0 (controller setting in periodic)
    m_RPMLastActiveSetPoint1 = m_RPMSetPoint1;
    m_RPMLastActiveSetPoint2 = m_RPMSetPoint2;
    m_RPMSetPoint1 = 0;
    m_RPMSetPoint2 = 0;

    // Force Dashboard update to prevent endless restart
    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPoint1);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPoint2);


    System.out.println("[SHOOTER] Deactivated (" + m_RPMSetPoint1 + ", " + m_RPMSetPoint2 + ")");
  }

/**
 * Stop with delay (seconds)
 * @param seconds Timer dutration in seconds 
 */
public void deactivate(double seconds){
  if(!m_delayedStopStarted){
    m_delayedStopDuration = seconds;
    m_delayedStopStarted = true;
    m_delayedStopTimer.reset();
    m_delayedStopTimer.start();
  }
}

// Return Shooter Status
public boolean isActive(){
  return m_activated;
}

// // Shooter is spinning at target speed!
public boolean isAtSpeed() {
  boolean retval = true; 
  // if( (Math.abs(m_encoder.getVelocity() - m_RPMSetPoint1) > Shooter.kRPMTolerance)
  //   && (Math.abs(m_encoder3.getVelocity() - m_RPMSetPoint2) <= Shooter.kRPMTolerance) ){
  //   retval = false;
  // }
  return retval;
}

  public String targetToString(Target target){
    String retval = "";
    
    switch(target){
      case LOW:
        retval = "LOW Goal";
        break;
      case HIGH:
        retval = "HIGH Goal";
        break;
      case DISCARD:
        retval = "Discard Cargo";
        break;
      case USER:
        retval = "USER Defined";
        break;
    }
    return retval;
  }

  public void setRPMTarget(Target target){ 
    double newSetPoint1 = 0;
    double newSetPoint2 = 0;
    
    // switch(target){
    //   case LOW:
    //     newSetPoint1 = Shooter.kShooter1RPM_LowGoal;
    //     newSetPoint2 = Shooter.kShooter2RPM_LowGoal;
    //     break;
    //   case HIGH:
    //     newSetPoint1 = Shooter.kShooter1RPM_HighGoal;
    //     newSetPoint2 = Shooter.kShooter2RPM_HighGoal;
    //     break;
    //   case DISCARD:
    //     newSetPoint1 = Shooter.kShooter1RPM_Discard;
    //     newSetPoint2 = Shooter.kShooter2RPM_Discard;
    //    break;
    //   case USER:
    //     newSetPoint1      = m_RPMInput1;
    //     newSetPoint2 = m_RPMInput2;
    //     break;
    // }
    // if((m_RPMSetPoint1 != newSetPoint1) ||  (m_RPMSetPoint2 != newSetPoint2) || (m_RPMLastActiveSetPoint1 != newSetPoint1) || (m_RPMLastActiveSetPoint2 != newSetPoint2)){
    //   System.out.println("[SHOOTER] " + targetToString(target) + " (" + newSetPoint1 + ", " + newSetPoint2 +")");
    // }

    // if(m_activated){
    //   if((m_RPMSetPoint1 != newSetPoint1) || (m_RPMSetPoint2 != newSetPoint2)){
    //     System.out.println("[SHOOTER] (Activated) New RPM (" + newSetPoint1 + ", " + newSetPoint2 + ")");
    //   }
    //   m_RPMSetPoint1      = newSetPoint1;
    //   m_RPMSetPoint2 = newSetPoint2;
    // } else {
    //   m_RPMLastActiveSetPoint1    = newSetPoint1;
    //   m_RPMLastActiveSetPoint2 = newSetPoint2;
    //   System.out.println("[SHOOTER] (Deactivated) Storing RPM (" + m_RPMLastActiveSetPoint1 + ", " + m_RPMLastActiveSetPoint2 +")");
    // }
  }

  // private Target getTarget(){
  //   return m_shooterTarget;
  // }
}