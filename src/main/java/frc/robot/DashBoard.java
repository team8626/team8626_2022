// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashBoard {

    enum StartPosition {
        TARMAC_1, TARMAC_2, TARMAC_3
    }

    enum AutoSelec {
        EXIT, TEST, EXAMPLE, GOAL_FIRST, COLLECT_FIRST
    }

    private static final Notifier m_thread = new Notifier(new dashboardThread());
	
	public static final boolean kEnableDashBoard = true;
	
    // private DesiredMode mCachedDesiredMode = null;
    // private StartingPosition mCachedStartingPosition = null;
    SendableChooser<StartPosition> m_startPositionChooser = new SendableChooser<>();
    SendableChooser<AutoSelec> m_autonomousModeChooser = new SendableChooser<>();

    static final double kShortInterval = .02;
    static final double kLongInterval = .5;
    
    static double m_shortOldTime = 0.0;
    static double m_longOldTime = 0.0;   

    /**
     * Class Constructor
     * Initialize the Dashboard with defaul values of "settable" inputs.
     */
    public DashBoard() {
        if(kEnableDashBoard){
            //SmartDashboard.putBoolean("Compressor ENABLE", true);
            //SmartDashboard.putBoolean("Limelight-LED Toggle", false);

            initStartupPostion();
            initAutonomousStrategy();

        }
        m_thread.startPeriodic(kShortInterval);    
        SmartDashboard.putString("Robot Status", "empty");   
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

    /** 
     * Initialize Robot Autonomous Strategy 
     */
    private void initAutonomousStrategy(){
        
        m_autonomousModeChooser.setDefaultOption("Exit", AutoSelec.EXIT);
        m_autonomousModeChooser.addOption("Example", AutoSelec.EXAMPLE);

        SmartDashboard.putData("Auto Mode", m_autonomousModeChooser);
        SmartDashboard.putNumber("Some Number", 1.45);
    }

    public void initShooterVoltage(){
        SmartDashboard.putNumber("Shooter_Voltage", 1.00);
    }


    public static void updateDashboard() {
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

    /**
     * Update values that need high frequency refresh.
     **/   
    private static void updateShortInterval() {
        // RobotContainer.swerve.dashboard();
        // RobotContainer.launcher.dashboard();
        // RobotContainer.tower.dashboard();
        // RobotContainer.intake.dashboard();
        // RobotContainer.indexer.dashboard();
        // RobotContainer.climber.dashboard();
        // SmartDashboard.putBoolean("Pressure SW",
        // RobotContainer.compressor.getPressureSwitchValue());
    }

    /**
     * Update values that need low frequency refresh.
     **/
    private static void updateLongInterval(){
        //SmartDashboard.putBoolean("Compressor On?", RobotContainer.pneumatics.compressor.enabled());
        
        //Can change to show a different message than "Yes" and "No"
        // SmartDashboard.putBoolean("Change Battery", Util.changeBattery());
    }


    // static int t = 0;
    // static boolean b = true;
    // 
    // public static void dashboardFlash(){
    //     //Flash a light on the dashboard so that you know that the dashboard is refreshing.
    //     if (t > 20) {
    //         t = 0;
    //         b = !b;
    //         SmartDashboard.putBoolean("Disabled Toggle", b);
    //     }
    //     t++;
    // }

    private static class dashboardThread implements Runnable {  
        @Override
        public void run() {
            updateDashboard();
        }
    }

 // MOVED TO AUTOMOMOUS... CAN BE DELETED?
    // /**
    // * Use this to pass the autonomous command to the main {@link Robot} class.
    // *
    // * @return the command to run in autonomous
    // */
    // public Command getStartCommand() {
    //     Command startCommand = null;
    //     startPosition = m_startPositionChooser.getselected();
    //     autoStart = m_autonomousModeChooser.getselected();
          
     
    //     switch (autoStart) {
    //         case EXIT: 
    //             System.out.println("Exit (Universal)");
    //             return new SequentialCommandGroup  (new FollowTrajectory("json", m_drivetrain));
    //             break;
        
    //         case EXAMPLE: switch (startPosition) {
    //             case TARMAC_1: 
    //                 System.out.println("Do example; Tarmac 1");
    //                 break;

    //             case TARMAC_2: 
    //                 System.out.println("Do example; Tarmac 2");
    //                 break;

    //             case TARMAC_3: 
    //                 System.out.println("Do example; Tarmac 3");
    //                 break;
    //         }
    //     }
    //     return startCommand;

  
    // }

}
