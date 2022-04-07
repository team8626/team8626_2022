package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
public class ShooterSubsystem extends SubsystemBase {

  // Shooter Motors
  private final CANSparkMax m_motorMain = new CANSparkMax(Shooter.kCANMotorShooterMain, MotorType.kBrushless);
  private final CANSparkMax m_motorSecondary = new CANSparkMax(Shooter.kCANMotorShooterSecondary, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_encoderMain       = m_motorMain.getEncoder();
  private final RelativeEncoder m_encoderSecondary  = m_motorSecondary.getEncoder();

  // PID Controllers
  private final SparkMaxPIDController m_pidControllerMain = m_motorMain.getPIDController();
  private final SparkMaxPIDController m_pidControllerSecondary = m_motorSecondary.getPIDController();

  private double m_pMain  = Shooter.kP_Main;
  private double m_iMain  = Shooter.kI_Main;
  private double m_dMain  = Shooter.kD_Main;
  private double m_izMain = Shooter.kIz_Main;
  private double m_ffMain = Shooter.kFF_Main;

  private double m_pSecondary  = Shooter.kP_Secondary;
  private double m_iSecondary  = Shooter.kI_Secondary;
  private double m_dSecondary  = Shooter.kD_Secondary;
  private double m_izSecondary = Shooter.kIz_Secondary;
  private double m_ffSecondary = Shooter.kFF_Secondary;

  private double m_minOutput = Shooter.kMinOutput;
  private double m_maxOutput = Shooter.kMaxOutput;

  // Store Velocity set by User on Dashboard
  private double m_RPMInputMain = Shooter.kShooterMainRPM_LowGoal;
  private double m_RPMInputSecondary = Shooter.kShooterSecondaryRPM_LowGoal;

   // Store SetPoint
   private double m_RPMSetPointMain = 0;
   private double m_RPMSetPointSecondary = 0;
   
   private double m_RPMLastActiveSetPointMain = m_RPMInputMain;
   private double m_RPMLastActiveSetPointSecondary = m_RPMInputSecondary;


  //  private double m_mainTuneTarget      = m_mainRPMTarget;
  //  private double m_secondaryTuneTarget = m_secondaryRPMTarget;

  // Velocity Actually set on the Controllers
  // private double m_mainRPMRequest      = 0;
  // private double m_secondaryRPMRequest = 0;

  // Shooter Targets
  public enum Target {LOW, HIGH, DISCARD, EMPTY};

  // Internal States
  private boolean m_activated;

  private SendableChooser<Target> m_targetChooser = new SendableChooser<>();
  private Target m_shooterTarget = Shooter.kDefaultTarget;

  private Timer m_delayedStopTimer = new Timer();
  private double m_delayedStopDuration = 1.0;
  private boolean m_delayedStopStarted = false;
  
  // Class Constructor
  public ShooterSubsystem() {

    // Set motor inverted or not...
    m_motorMain.setInverted(true);
    m_motorSecondary.setInverted(false);

    // Set PID coefficients
    m_motorMain.restoreFactoryDefaults();
    m_pidControllerMain.setP(m_pMain);
    m_pidControllerMain.setI(m_iMain);
    m_pidControllerMain.setD(m_dMain);
    m_pidControllerMain.setIZone(m_izMain);
    m_pidControllerMain.setFF(m_ffMain);
    m_pidControllerMain.setOutputRange(m_minOutput, m_maxOutput);

    m_motorSecondary.restoreFactoryDefaults();
    m_pidControllerSecondary.setP(m_pSecondary);
    m_pidControllerSecondary.setI(m_iSecondary);
    m_pidControllerSecondary.setD(m_dSecondary);
    m_pidControllerSecondary.setIZone(m_izSecondary);
    m_pidControllerSecondary.setFF(m_ffSecondary);
    m_pidControllerSecondary.setOutputRange(m_minOutput, m_maxOutput);

    // Initialize states
    //this.deactivate();
  }  

  // Initialize Dashboard
  public void initDashboard(){
    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("MAIN P Gain", m_pMain);
    SmartDashboard.putNumber("MAIN I Gain", m_iMain);
    SmartDashboard.putNumber("MAIN D Gain", m_dMain);
    SmartDashboard.putNumber("MAIN I Zone", m_izMain);
    SmartDashboard.putNumber("MAIN Feed Forward", m_ffMain);

    SmartDashboard.putNumber("SECONDARY P Gain", m_pSecondary);
    SmartDashboard.putNumber("SECONDARY I Gain", m_iSecondary);
    SmartDashboard.putNumber("SECONDARY D Gain", m_dSecondary);
    SmartDashboard.putNumber("SECONDARY I Zone", m_izSecondary);
    SmartDashboard.putNumber("SECONDARY Feed Forward", m_ffSecondary);

    SmartDashboard.putNumber("Max Output", m_minOutput);
    SmartDashboard.putNumber("Min Output", m_maxOutput);

    // Display Wheel RPM on Dashboard
    SmartDashboard.putNumber("Input Wheel RPM", m_RPMInputMain);
    SmartDashboard.putNumber("Input Back Spin RPM", m_RPMInputSecondary);

    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPointMain);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPointSecondary);

    SmartDashboard.putNumber("Actual Wheel RPM", m_encoderMain.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoderSecondary.getVelocity());

    SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Taget Selection
  //   //m_targetChooser.addOption(targetToString(Target.EMPTY), Target.EMPTY);
  //   m_targetChooser.addOption(targetToString(Target.LOW), Target.LOW);
  //   m_targetChooser.addOption(targetToString(Target.HIGH), Target.HIGH);
  //   //m_targetChooser.addOption(targetToString(Target.DISCARD), Target.DISCARD);
  //   m_targetChooser.setDefaultOption(targetToString(m_shooterTarget), m_shooterTarget);
  //   SmartDashboard.putData("SHOOTER Target", m_targetChooser);
  }

  // Update Dashboard (Called Periodically)
  public void updateDashboard(){
    // Read SmartDashboard PID Coefficients
    double pMain = SmartDashboard.getNumber("MAIN P Gain", 0);
    double iMain = SmartDashboard.getNumber("MAIN I Gain", 0);
    double dMain = SmartDashboard.getNumber("MAIN D Gain", 0);
    double izMain = SmartDashboard.getNumber("MAIN I Zone", 0);
    double ffMain  = SmartDashboard.getNumber("MAIN Feed Forward", 0);

    double pSec = SmartDashboard.getNumber("SECONDARY P Gain", 0);
    double iSec = SmartDashboard.getNumber("SECONDARY I Gain", 0);
    double dSec = SmartDashboard.getNumber("SECONDARY D Gain", 0);
    double izSec = SmartDashboard.getNumber("SECONDARY I Zone", 0);
    double ffSec = SmartDashboard.getNumber("SECONDARY Feed Forward", 0);

    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    double setpoint_main      = SmartDashboard.getNumber("SetPoint Wheel RPM", 0);
    double setpoint_secondary = SmartDashboard.getNumber("SetPoint Back Spin RPM", 0);
    
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((pMain != m_pMain))   { m_pidControllerMain.setP(pMain); m_pMain = pMain; }
      if((iMain != m_iMain))   { m_pidControllerMain.setI(iMain); m_iMain = iMain; }
      if((dMain != m_dMain))   { m_pidControllerMain.setD(dMain); m_dMain = dMain; }
      if((izMain != m_izMain)) { m_pidControllerMain.setIZone(izMain); m_izMain = izMain; }
      if((ffMain != m_ffMain)) { m_pidControllerMain.setFF(ffMain); m_ffMain = ffMain; }

      if((pSec != m_pSecondary))   { m_pidControllerSecondary.setP(pSec); m_pSecondary = pSec; }
      if((iSec != m_iSecondary))   { m_pidControllerSecondary.setI(iSec); m_iSecondary = iSec; }
      if((dSec != m_dSecondary))   { m_pidControllerSecondary.setD(dSec); m_dSecondary = dSec; }
      if((izSec != m_izSecondary)) { m_pidControllerSecondary.setIZone(izSec); m_izSecondary = izSec; }
      if((ffSec != m_ffSecondary)) { m_pidControllerSecondary.setFF(ffSec); m_ffSecondary = ffSec; }

      if((max != m_maxOutput) || (min != m_minOutput)) { 
        m_pidControllerMain.setOutputRange(min, max); 
        m_pidControllerSecondary.setOutputRange(min, max); 
        m_minOutput = min; 
        m_maxOutput = max; 
      }

      // Read User input for RPM and store them
      if((setpoint_main != m_RPMSetPointMain))   { m_RPMSetPointMain = setpoint_main; }
      if((setpoint_secondary != m_RPMSetPointSecondary))   { m_RPMSetPointSecondary = setpoint_secondary; }

      
    // Update Shooter Speed based on Shuffleboard
    // Is selection changed, use the selection.
    // Otherwise, use the tuning values if changed.
    Target newTarget = m_targetChooser.getSelected();
    if(newTarget != m_shooterTarget){
  //     if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target: " + targetToString(newTarget)); }
  //     m_shooterTarget = newTarget;
  //     setRPMTarget(m_shooterTarget);
  //     m_mainTuneTarget = m_mainRPMTarget;
  //     m_secondaryTuneTarget = m_secondaryRPMTarget;
    } else {
      // Adjust Wheel Speed if changed

  //     if((main_rot != m_mainTuneTarget)) {
  //       if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target MAIN (" + m_mainTuneTarget + " -> " + main_rot +")"); }
  //       m_mainTuneTarget = main_rot; 
  //       m_mainRPMTarget = m_mainTuneTarget;
  //     };
  //     if((secondary_rot != m_secondaryTuneTarget)) { 
  //       if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target SECONDARY (" + m_secondaryTuneTarget + " -> " + secondary_rot +")"); }
  //       m_secondaryTuneTarget = secondary_rot; 
  //       m_secondaryRPMTarget = m_secondaryTuneTarget; 
      // };

  //     // m_targetChooser.setDefaultOption(targetToString(Target.EMPTY), Target.EMPTY);
  //     // SmartDashboard.putData("SHOOTER Target", m_targetChooser);
    } 

    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Update Speed values ( User, SetPoint, Actual)
    SmartDashboard.putNumber("Input Wheel RPM", m_RPMInputMain);
    SmartDashboard.putNumber("Input Back Spin RPM", m_RPMInputSecondary);

    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPointMain);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPointSecondary);

    SmartDashboard.putNumber("Actual Wheel RPM", m_encoderMain.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoderSecondary.getVelocity());
    
  //   SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

  }

  // Periodic Updates
  @Override
  public void periodic(){
    // Keep Adjusting RPM Setpoint
    m_pidControllerMain.setReference(m_RPMSetPointMain, CANSparkMax.ControlType.kVelocity);
    m_pidControllerSecondary.setReference(m_RPMSetPointSecondary, CANSparkMax.ControlType.kVelocity);

    // Check the delayed Stop timer
    if(m_delayedStopStarted && (m_delayedStopTimer.hasElapsed(m_delayedStopDuration))){
      m_delayedStopStarted = false;
      m_delayedStopTimer.stop();
      this.deactivate();
    }
  }

  public void activate(){
    m_pidControllerMain.setReference(m_RPMLastActiveSetPointMain, CANSparkMax.ControlType.kVelocity);
    m_pidControllerSecondary.setReference(m_RPMLastActiveSetPointSecondary, CANSparkMax.ControlType.kVelocity);
    m_activated = true;
    if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] Activated"); }

  }

  public void deactivate(){
     m_activated = false;
    
    // Set new setpoint to 0 (controller setting in periodic)
    m_RPMLastActiveSetPointMain = m_RPMSetPointMain;
    m_RPMLastActiveSetPointSecondary = m_RPMSetPointSecondary;
    m_RPMSetPointMain = 0;
    m_RPMSetPointSecondary = 0;

    // Force Dashboard update to prevent endless restart
    SmartDashboard.putNumber("SetPoint Wheel RPM", m_RPMSetPointMain);
    SmartDashboard.putNumber("SetPoint Back Spin RPM", m_RPMSetPointSecondary);


    if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] Deactivated"); }
  }

/**
 * Stop with delay (seconds)
 * @param seconds Timer dutration in seconds 
 */
public void deactivate(double seconds){
  m_delayedStopDuration = seconds;
  m_delayedStopStarted = true;
  m_delayedStopTimer.reset();
  m_delayedStopTimer.start();
}

// Return Shooter Status
public boolean isActive(){
  return m_activated;
}

// // Shooter is spinning at target speed!
public boolean isAtSpeed() {
  boolean retval = true;  // TODO Always True since could not get shooter at setpoint...
//   if( (Math.abs(m_encoderMain.getVelocity() - m_mainRPMRequest) > Shooter.kRPMTolerance)
//     && (Math.abs(m_encoderSecondary.getVelocity() - m_secondaryRPMRequest) <= Shooter.kRPMTolerance) ){
//     retval = false;
//   }
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
      case EMPTY:
        retval = "---";
        break;
    }
    return retval;
  }

  public void setRPMTarget(Target target){ 
    // TODO Currently doing nothing since Shooter not tuned

  //   switch(target){
  //     default:
  //     case LOW:
  //       m_mainRPMTarget = Shooter.kShooterMainRPM_LowGoal;
  //       m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_LowGoal;
  //       if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] LOW (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }
  //       break;
  //     case HIGH:
  //       m_mainRPMTarget = Shooter.kShooterMainRPM_HighGoal;
  //       m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_HighGoal;
  //       if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] HIGH (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }
  //       break;
  //     case DISCARD:
  //       m_mainRPMTarget = Shooter.kShooterMainRPM_Discard;
  //       m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_Discard;
  //       if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] DISCARD (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }

  //       break;
  //   }
  }

  public Target getTarget(){
    return m_shooterTarget;
  }
}
