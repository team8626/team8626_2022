package frc.robot.subsystems;
  
  import edu.wpi.first.math.geometry.Pose2d;
  import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
  import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
 import edu.wpi.first.wpilibj.ADXRS450_Gyro;
 import edu.wpi.first.wpilibj.Encoder;
 import edu.wpi.first.wpilibj.drive.DifferentialDrive;
 // import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
 import edu.wpi.first.wpilibj.interfaces.Gyro;
 import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
 import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveTrain;
import edu.wpi.first.wpilibj.AnalogGyro;

/* Main class for handling Drivetrain 
**
**
*/

 public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax m_motorFrontLeft = new CANSparkMax(DriveTrain.kCANMotorFL, MotorType.kBrushed);
    private final CANSparkMax m_motorRearLeft = new CANSparkMax(DriveTrain.kCANMotorRL, MotorType.kBrushed);
    private final CANSparkMax m_motorFrontRight = new CANSparkMax(DriveTrain.kCANMotorFR, MotorType.kBrushed);
    private final CANSparkMax m_motorRearRight = new CANSparkMax(DriveTrain.kCANMotorRR, MotorType.kBrushed);

    private final MotorControllerGroup m_motorControllerLeft = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);
    private final MotorControllerGroup m_motorControllerRight = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);
  
    private final DifferentialDrive m_drive = new DifferentialDrive (m_motorControllerLeft, m_motorControllerRight);
  
    // The left-side drive encoder
    private final Encoder m_leftEncoder =
          new Encoder(
              DriveTrain.kLeftEncoderPorts[0],
              DriveTrain.kLeftEncoderPorts[1],
              DriveTrain.kLeftEncoderReversed);

      // The right-side drive encoder
      private final Encoder m_rightEncoder =
          new Encoder(
            DriveTrain.kRightEncoderPorts[0],
            DriveTrain.kRightEncoderPorts[1],
            DriveTrain.kRightEncoderReversed);

      //  Gyro Sensor
      private final Gyro m_gyro = new AnalogGyro(0);
      
      // Odometry class for tracking robot location
      private final DifferentialDriveOdometry m_odometry;

      public DriveSubsystem() {
        // Sets distance for per pulse for the encoders
        m_leftEncoder.setDistancePerPulse(DriveTrain.kEncoderMetersPerPulse);
        m_rightEncoder.setDistancePerPulse(DriveTrain.kEncoderMetersPerPulse);  

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
        
      }  
      
      private void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();  
      }
/**   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);

  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_motorControllerLeft.setVoltage(leftVolts);
    m_motorControllerRight.setVoltage(rightVolts);
    m_drive.feed();
  }



}
