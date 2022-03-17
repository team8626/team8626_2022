// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

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
        public static int kCANMotorFL = 1;  // USING SPARKMAX
        public static int kCANMotorRL = 2;  // USING SPARKMAX
        public static int kCANMotorFR = 3;  // USING SPARKMAX
        public static int kCANMotorRR = 4;  // USING SPARKMAX
        // Encoder Ports
        public static int[] kLeftEncoderPorts = {0,1};
        public static int[] kRightEncoderPorts = {2,3};

        public static boolean kLeftEncoderReversed = false;
        public static boolean kRightEncoderReversed = true;

        public static double kWheelDiameter = Units.inchesToMeters(6.0) ;
        public static int kEncoderPulsesPerRev = 256;
        public static double kEncoderMetersPerPulse = kWheelDiameter * Math.PI / kEncoderPulsesPerRev;

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        //TODO update placeholder values
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
        public static int kCANMotorIntake = 5;  // USING VICTOR SPX

        // Pneumatic Controller Address
        public static int kPneumaticModuleID = 0;

        public static int kcylinderLeftExtend   = 0;
        public static int kcylinderLeftRetract  = 1;
        public static int kcylinderRightExtend  = 2;
        public static int kcylinderRightRetract = 3;

        // Intake Parameters
        public static double kMotorIntakePower = 1.0; // Maximum intake motor power [0.0:1.0]
    }

    // Storage subsystem constants
    public final static class Storage {
        // CAN Bus Addresses for Motors
        public static int kCANMotorStorageFront = 6;  // USING VICTOR SPX
        public static int kCANMotorStorageBack = 7;   // USING VICTOR SPX

        // i2C Addresses for Color Sensors
        public static I2C.Port kI2CColorSensorPortFront = I2C.Port.kOnboard;
        public static I2C.Port kI2CColorSensorPortBack  = I2C.Port.kMXP;

        // Timeout for Storage Units
        public static int kTimeoutStorageUnit= 10;
    }

    // Cargo constants
    public final static class Cargo {
        // Colors
        public static Color kBlue = new Color(0.2, 0.45, 0.34);
        public static Color kRed   = new Color(0.4, 0.4, 0.2);
    }

    // Climber subsystem constants
    public final static class Climber {
        // CAN Bus Addresses for Motors
        public static int kCANMotorClimberLeft  = 8;   // USING VICTOR SPX
        public static int kCANMotorClimberRight = 9;   // USING VICTOR SPX
    }

    // Shooter subsystem constants
    public final static class Shooter {
        // CAN Bus Addresses for Motor
        public static int kCANMotorShooter  = 10;      // USING VICTOR SPX

        // Predefined Voltage for Shooter Motor
        public static double kShooterVoltageHighGoal  = 9.0;
        public static double kShooterVoltageLowGoal   = 6.0;
        public static double kShooterVoltageDiscard   = 4.0;
        
        // Time for the shooter to reach stable speed.
        public static int kShooterSpinSeconds = 2;
    }
    
    // Controller station constants
    public final static class Controller {
        public static int kPS4Port = 0;
        public static int kGamepadPort = 1;
    }
}