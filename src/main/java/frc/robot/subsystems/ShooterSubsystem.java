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
import com.revrobotics.SparkMaxAlternateEncoder.Type;

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
  private final RelativeEncoder m_encoderMain       = m_motorMain.getAlternateEncoder(Type.kQuadrature, Shooter.kEncoderCountPerRev);
  private final RelativeEncoder m_encoderSecondary  = m_motorSecondary.getAlternateEncoder(Type.kQuadrature, Shooter.kEncoderCountPerRev);

  // PID Controllers
  private final SparkMaxPIDController m_pidControllerMain = m_motorMain.getPIDController();
  private final SparkMaxPIDController m_pidControllerSecondary = m_motorSecondary.getPIDController();

  private double m_p  = Shooter.kP;
  private double m_i  = Shooter.kI;
  private double m_d  = Shooter.kD;
  private double m_iz = Shooter.kIz;
  private double m_ff = Shooter.kFF;
  private double m_minOutput = Shooter.kMinOutput;
  private double m_maxOutput = Shooter.kMaxOutput;

  // Store Velocity as set
  private double m_mainRPMTarget      = Shooter.kShooterMainRPM_LowGoal;
  private double m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_LowGoal;

   // Memorize last Tuned value
   private double m_mainTuneTarget      = m_mainRPMTarget;
   private double m_secondaryTuneTarget = m_secondaryRPMTarget;

  // Velocity Actually set on the Controllers
  private double m_mainRPMRequest      = 0;
  private double m_secondaryRPMRequest = 0;

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
    m_motorMain.setInverted(false);
    m_motorSecondary.setInverted(true);

    // Set PID coefficients
    m_motorMain.restoreFactoryDefaults();
    m_pidControllerMain.setP(m_p);
    m_pidControllerMain.setI(m_i);
    m_pidControllerMain.setD(m_d);
    m_pidControllerMain.setIZone(m_iz);
    m_pidControllerMain.setFF(m_ff);
    m_pidControllerMain.setOutputRange(m_minOutput, m_maxOutput);

    m_motorSecondary.restoreFactoryDefaults();
    m_pidControllerSecondary.setP(m_p);
    m_pidControllerSecondary.setI(m_i);
    m_pidControllerSecondary.setD(m_d);
    m_pidControllerSecondary.setIZone(m_iz);
    m_pidControllerSecondary.setFF(m_ff);
    m_pidControllerSecondary.setOutputRange(m_minOutput, m_maxOutput);

    // Initialize states
    this.deactivate();
  }  

  // Initialize Dashboard
  public void initDashboard(){
    // Display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", m_p);
    SmartDashboard.putNumber("I Gain", m_i);
    SmartDashboard.putNumber("D Gain", m_d);
    SmartDashboard.putNumber("I Zone", m_iz);
    SmartDashboard.putNumber("Feed Forward", m_ff);
    SmartDashboard.putNumber("Max Output", m_minOutput);
    SmartDashboard.putNumber("Min Output", m_maxOutput);

    // Display Wheel RPM on Dashboard
    SmartDashboard.putNumber("Tuned Wheel RPM", m_mainTuneTarget);
    SmartDashboard.putNumber("Tuned Back Spin RPM", m_secondaryTuneTarget);
    SmartDashboard.putNumber("Set Wheel RPM", 0);
    SmartDashboard.putNumber("Set Back Spin RPM", 0);
    SmartDashboard.putNumber("Actual Wheel RPM", m_encoderMain.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoderSecondary.getVelocity());
    SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Taget Selection
    //m_targetChooser.addOption(targetToString(Target.EMPTY), Target.EMPTY);
    m_targetChooser.addOption(targetToString(Target.LOW), Target.LOW);
    m_targetChooser.addOption(targetToString(Target.HIGH), Target.HIGH);
    //m_targetChooser.addOption(targetToString(Target.DISCARD), Target.DISCARD);
    m_targetChooser.setDefaultOption(targetToString(m_shooterTarget), m_shooterTarget);
    SmartDashboard.putData("SHOOTER Target", m_targetChooser);
  }

  // Update Dashboard (Called Periodically)
  public void updateDashboard(){
    // Read SmartDashboard PID Coefficients
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double main_rot = SmartDashboard.getNumber("Tuned Wheel RPM", 0);
    double secondary_rot = SmartDashboard.getNumber("Tuned Back Spin RPM", 0);
    
    if(m_activated) {
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != m_p)) { m_pidControllerMain.setP(p); m_pidControllerSecondary.setP(p); m_p = p; }
      if((i != m_i)) { m_pidControllerMain.setI(i); m_pidControllerSecondary.setI(i); m_i = i; }
      if((d != m_d)) { m_pidControllerMain.setD(d); m_pidControllerSecondary.setD(d); m_d = d; }
      if((iz != m_iz)) { m_pidControllerMain.setIZone(iz); m_pidControllerSecondary.setIZone(iz); m_iz = iz; }
      if((ff != m_ff)) { m_pidControllerMain.setFF(ff); m_pidControllerSecondary.setFF(ff); m_ff = ff; }
      if((max != m_maxOutput) || (min != m_minOutput)) { 
        m_pidControllerMain.setOutputRange(min, max); 
        m_pidControllerSecondary.setOutputRange(min, max); 
        m_minOutput = min; m_maxOutput = max; 
      }
    }
    
    // Update Shooter Speed based on Shuffleboard
    // Is selection changed, use the selection.
    // Otherwise, use the tuning values if changed.
    Target newTarget = m_targetChooser.getSelected();
    if(newTarget != m_shooterTarget){
      if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target: " + targetToString(newTarget)); }
      m_shooterTarget = newTarget;
      setRPMTarget(m_shooterTarget);
      m_mainTuneTarget = m_mainRPMTarget;
      m_secondaryTuneTarget = m_secondaryRPMTarget;
    } else {
      // Adjust Wheel Speed if changed

      if((main_rot != m_mainTuneTarget)) {
        if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target MAIN (" + m_mainTuneTarget + " -> " + main_rot +")"); }
        m_mainTuneTarget = main_rot; 
        m_mainRPMTarget = m_mainTuneTarget;
      };
      if((secondary_rot != m_secondaryTuneTarget)) { 
        if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] New Target SECONDARY (" + m_secondaryTuneTarget + " -> " + secondary_rot +")"); }
        m_secondaryTuneTarget = secondary_rot; 
        m_secondaryRPMTarget = m_secondaryTuneTarget; 
      };

      // m_targetChooser.setDefaultOption(targetToString(Target.EMPTY), Target.EMPTY);
      // SmartDashboard.putData("SHOOTER Target", m_targetChooser);
    } 

    // Set Speed on the Controllers
    if(m_activated) {
      m_mainRPMRequest = m_mainRPMTarget;
      m_secondaryRPMRequest = m_secondaryRPMTarget;
    } else {
      m_mainRPMRequest = 0;
      m_secondaryRPMRequest = 0;
    }

    // Set Speed in the Controllers using the dashboard values
    m_pidControllerMain.setReference(m_mainRPMRequest, CANSparkMax.ControlType.kVelocity);
    m_pidControllerSecondary.setReference(m_secondaryRPMRequest, CANSparkMax.ControlType.kVelocity);

    // General Status
    SmartDashboard.putBoolean("SHOOTER", m_activated);

    // Update Speed values ( Requested, Tuned, Actual)
    SmartDashboard.putNumber("Set Wheel RPM", m_mainRPMRequest);
    SmartDashboard.putNumber("Set Back Spin RPM", m_secondaryRPMRequest);
    SmartDashboard.putNumber("Tuned Wheel RPM", m_mainTuneTarget);
    SmartDashboard.putNumber("Tuned Back Spin RPM", m_secondaryTuneTarget);
    SmartDashboard.putNumber("Actual Wheel RPM", m_encoderMain.getVelocity());
    SmartDashboard.putNumber("Actual Back Spin RPM", m_encoderSecondary.getVelocity());
    SmartDashboard.putBoolean("SPEED OK ", this.isAtSpeed());

  }

  // Periodic Updates
  @Override
  public void periodic(){
    // Update Target RPMon motors
    m_pidControllerMain.setReference(m_mainRPMTarget, CANSparkMax.ControlType.kVelocity);
    m_pidControllerSecondary.setReference(m_secondaryRPMTarget, CANSparkMax.ControlType.kVelocity);

    // Check the delayed Stop timer
    if(m_delayedStopStarted && (m_delayedStopTimer.hasElapsed(m_delayedStopDuration))){
      m_delayedStopStarted = false;
      m_delayedStopTimer.stop();
      this.deactivate();
    }
  }

  public void activate(){
    m_activated = true;

    // Set Requested RPM (controller setting in periodic)
    m_mainRPMRequest = m_mainRPMTarget;
    m_secondaryRPMRequest = m_secondaryRPMTarget;

    if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] Activated"); }
  }

  public void deactivate(){
    m_activated = false;
    
    // Set Requested RPM to 0 (controller setting in periodic)
    m_mainRPMRequest = 0;
    m_secondaryRPMRequest = 0;

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

  public boolean isActive(){
    return m_activated;
  }

  // Shooter is spinning at target speed!
  public boolean isAtSpeed() {
    boolean retval = false;
    if( (Math.abs(m_encoderMain.getVelocity() - m_mainRPMRequest) <= Shooter.kRPMTolerance)
      && (Math.abs(m_encoderSecondary.getVelocity() - m_secondaryRPMRequest) <= Shooter.kRPMTolerance) ){
      retval = true;
    }
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
    switch(target){
      default:
      case LOW:
        m_mainRPMTarget = Shooter.kShooterMainRPM_LowGoal;
        m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_LowGoal;
        if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] LOW (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }
        break;
      case HIGH:
        m_mainRPMTarget = Shooter.kShooterMainRPM_HighGoal;
        m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_HighGoal;
        if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] HIGH (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }
        break;
      case DISCARD:
        m_mainRPMTarget = Shooter.kShooterMainRPM_Discard;
        m_secondaryRPMTarget = Shooter.kShooterSecondaryRPM_Discard;
        if(RobotBase.isSimulation()){ System.out.println("[SHOOTER] DISCARD (" + m_mainRPMTarget + ", " + m_secondaryRPMTarget +")"); }

        break;
    }
  }

  public Target getTarget(){
    return m_shooterTarget;
  }
}
