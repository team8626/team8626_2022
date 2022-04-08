// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // driveTrain subsystem constants
    public final static class DriveTrain {
        // CAN Bus addresses for motors
        public static int kCANMotorFL = 3;  // USING SPARKMAX
        public static int kCANMotorRL = 1;  // USING SPARKMAX
        public static int kCANMotorFR = 4;  // USING SPARKMAX
        public static int kCANMotorRR = 2;  // USING SPARKMAX

        // Encoder Ports
        public static int[] kLeftEncoderPorts = {0,1};
        public static int[] kRightEncoderPorts = {2,3};

        public static boolean kLeftEncoderReversed = false;
        public static boolean kRightEncoderReversed = true;

        // P{ower Moultiplicators
        public static double kPowerRatioLowSpeed = 0.5;
        public static double kPowerRatioHighSpeed = 1.0;

        // Drivetrain Characteristics
        public static double kWheelDiameter = Units.inchesToMeters(6.0) ;
        public static int kEncoderPulsesPerRev = 256;
        public static double kEncoderMetersPerPulse = kWheelDiameter * Math.PI / kEncoderPulsesPerRev;

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kMaxAvailableVoltage = 10.5; // Assumes Battery "sag" for PID/Ramsete Controllers

        // TODO update placeholder values _ NEED TO DO CHARACTERIZATION
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double kTrackwidthMeters = Units.inchesToMeters(21.75);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    // Intake subsystem constants
    public final static class Intake {
        // CAN Bus Addresses for Motor
        public static int kCANMotorIntake = 7;  // USING VICTOR SPX

        // Pneumatic Controller Address
        public static int kPneumaticModuleID = 0;

        public static int kCylinderExtend   = 0;
        public static int kCylinderRetract  = 1;

        // Intake Parameters
        public static double kMotorIntakePower = 1.0; // Maximum intake motor power [0.0:1.0]
    }

    // Storage subsystem constants
    public final static class Storage {
        // CAN Bus Addresses for Motors
        public static int kCANMotorStorageFront = 10;   // USING VICTOR SPX
        public static int kCANMotorStorageBack  =  6;   // USING VICTOR SPX

        // i2C Addresses for Color Sensors
        public static I2C.Port kI2CColorSensorPortFront = I2C.Port.kOnboard;
        public static I2C.Port kI2CColorSensorPortBack  = I2C.Port.kMXP;

        // Timeout for Storage Units
        public static int kTimeoutStorageUnit= 5;

        // USing Color Sensors
        public static boolean kIsUsingColorSensors = true;
    }

    // Cargo constants
    public final static class Cargo {
        // Colors
        public static Color kBlue = new Color(0.143, 0.427, 0.429);
        public static Color kRed   = new Color(0.561, 0.232, 0.114);
        public static Color kDummyGreen = new Color(0.197, 0.561, 0.240);
        public static Color kDummyYellow = new Color(0.361, 0.524, 0.113);
    }

    // Climber subsystem constants
    public final static class Climber {
        // CAN Bus Addresses for Motors
        public static int kCANMotorClimberLeft  = 11;   // USING VICTOR SPX
        public static int kCANMotorClimberRight = 22;   // USING VICTOR SPX
    }

    // Shooter subsystem constants
    public final static class Shooter {
         // CAN Bus Addresses for Motor
         public static int kCANMotorShooterMain       =  9;      // USING SPARKMAX & NEO
         public static int kCANMotorShooterSecondary  = 13;      // USING SPARKMAX & NEO

         // Predefined Voltage for Shooter Motor
        //  public static double kShooterVoltageMainHighGoal  = 9.0;
        //  public static double kShooterVoltageMainLowGoal   = 5.5;
        //  public static double kShooterVoltageMainDiscard   = 4.0;
 
         // Encoder Specifications
         // public static int kEncoderCountPerRev = 42; // NEO and NEO 550 Encoder Count per Revolution
 
         //Predefined RPM for sooter motors
         public static double kShooterMainRPM_LowGoal       = 500;
         public static double kShooterSecondaryRPM_LowGoal  = 2000;

         public static double kShooterMainRPM_HighGoal      = 1000;
         public static double kShooterSecondaryRPM_HighGoal = 4000;
        
         public static double kShooterMainRPM_Discard       =  400;
         public static double kShooterSecondaryRPM_Discard  = -200;

         public static double kRPMTolerance = 0.01;
 
         // PID Loop Coefficients
         public static double kP_Main = 0.00006; 
         public static double kI_Main = 0;
         public static double kD_Main = 0; 
         public static double kIz_Main = 0; 
         public static double kFF_Main = 0.000019; 

         public static double kP_Secondary = 0.000002; 
         public static double kI_Secondary = 0;
         public static double kD_Secondary = 0; 
         public static double kIz_Secondary = 0; 
         public static double kFF_Secondary = 0.000019; 

         public static double kMaxOutput = 1; 
         public static double kMinOutput = -1;
  
        public static int kShooterSpinSeconds = 5;

        public static ShooterSubsystem.Target kDefaultTarget = ShooterSubsystem.Target.LOW;

    }
    
    // Controller station constants
    public final static class Controller {
        public static int kJoystickPort = 0;
        public static int kGamepadPort  = 1;
    }
}