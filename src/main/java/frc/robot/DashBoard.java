// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class DashBoard {

    enum StartPosition {
        TARMAC_1, TARMAC_2, TARMAC_3
    }

    enum AutoSelec {
        SHOOT_AND_MOVE, SHOOT_AND_MOVE_METERS, COLLECT_AND_SHOOT2, EXIT, TEST, EXAMPLE, GOAL_FIRST, COLLECT_FIRST
    }

    private static final Notifier m_thread = new Notifier(new dashboardThread());
	
	public static final boolean kEnableDashBoard = true;
	
    // private DesiredMode mCachedDesiredMode = null;
    // private StartingPosition mCachedStartingPosition = null;
    SendableChooser<StartPosition> m_startPositionChooser = new SendableChooser<>();
    SendableChooser<AutoSelec> m_autonomousModeChooser = new SendableChooser<>();

    static final double kShortInterval = .02;
    static final double kLongInterval  = .5;
    
    static double m_shortOldTime = 0.0;
    static double m_longOldTime  = 0.0;   

    // Subsystems
    private static IntakeSubsystem m_intake;
    private static StorageSubsystem m_storage;
    private static ShooterSubsystem m_shooter;

    /**
     * Class Constructor
     * Initialize the Dashboard with defaul values of "settable" inputs.
     */
    public DashBoard(IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {
        if(kEnableDashBoard){
            //SmartDashboard.putBoolean("Compressor ENABLE", true);
            //SmartDashboard.putBoolean("Limelight-LED Toggle", false);

            initAutonomousStrategy();
            initStartupPostion();
        }
        m_thread.startPeriodic(kShortInterval);

        m_intake = intake;
        m_storage = storage;
        m_shooter = shooter;

        initSubsystems();
    }

    /** 
     * Initialize Robot Autonomous Strategy 
     */
    private void initAutonomousStrategy(){
        
        m_autonomousModeChooser.addOption("Exit", AutoSelec.EXIT);
        m_autonomousModeChooser.setDefaultOption("Shoot and Move", AutoSelec.SHOOT_AND_MOVE);
        m_autonomousModeChooser.addOption("Shoot and Move Meters", AutoSelec.SHOOT_AND_MOVE_METERS);
        m_autonomousModeChooser.addOption("Collect and Shoot 2", AutoSelec.COLLECT_AND_SHOOT2);

        SmartDashboard.putData("Auto Mode", m_autonomousModeChooser);
    }
    
    /** 
     * Initialize Robot Starting Position
     */
    private void initStartupPostion(){
       
        m_startPositionChooser.setDefaultOption("Top Tarmac, Up", StartPosition.TARMAC_1);
        m_startPositionChooser.addOption("Top Tarmac, Low", StartPosition.TARMAC_2);
        m_startPositionChooser.addOption("Bottom Tarmac, Left", StartPosition.TARMAC_3);

        SmartDashboard.putData("Starting Position", m_startPositionChooser);
    }

    /** 
     * Returns Selected Robot Start Position
     */
    public StartPosition getStartPosition() {
        return m_startPositionChooser.getSelected();
    }
 
    /** 
     * Returns Selected Robot Autonomous Startup Mode
     */  
    public AutoSelec getAutoMode() {
        return m_autonomousModeChooser.getSelected();
    }

    private static void updateDashboard() {
        double time = Timer.getFPGATimestamp();
        if (kEnableDashBoard) {
            if ((time - m_shortOldTime) > kShortInterval) {
                m_shortOldTime = time;
                updateShortInterval();
            }
            if ((time - m_longOldTime) > kLongInterval) {
                //Thing that should be updated every LONG_DELAY
                m_longOldTime = time;
                updateShortInterval();
                updateLongInterval();
            }
        }
    }

    // Initialize Dashboard foe all subsystems.
    private static void initSubsystems() {
        SmartDashboard.putString("Robot Status", "---STARTING---");   

        //m_drivetrain.initDashboard();
        m_intake.initDashboard();
        m_storage.initDashboard();
        m_storage.getFrontUnit().initDashboard();
        m_storage.getBackUnit().initDashboard();
        m_shooter.initDashboard();
        //m_climber.initDashboard();
    }

    // Update values that need high frequency refresh.
    private static void updateShortInterval() {

        //m_drivetrain.updateDashboard();
         m_intake.updateDashboard();
         m_storage.updateDashboard();
         m_storage.getFrontUnit().updateDashboard();
         m_storage.getBackUnit().updateDashboard();
         m_shooter.updateDashboard();
         //m_climber.updateDashboard();

         // Pulsing to indicate Dashboard is updated
         dashboardFlash();
    }

    // Update values that need low frequency refresh.
    private static void updateLongInterval(){
        //SmartDashboard.putBoolean("Compressor On?", RobotContainer.pneumatics.compressor.enabled());
        
        // Can change to show a different message than "Yes" and "No"
        // SmartDashboard.putBoolean("Change Battery", Util.changeBattery());
    }


    //Flash a light on the dashboard so that you know that the dashboard is refreshing.
    static int t = 0;
    static boolean b = true;
    public static void dashboardFlash(){
        if (t > 20) {
            t = 0;
            b = !b;
            SmartDashboard.putBoolean("Pulse", b);
        }
        t++;
    }

    private static class dashboardThread implements Runnable {  
        @Override
        public void run() {
            updateDashboard();
        }
    }
}
