package frc.robot;

// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import frc.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DashBoard {

//     enum StartPosition {
//         TARMAC_1, TARMAC_2, TARMAC_3
//     }


//     private static final Notifier m_thread = new Notifier(new dashboardThread());
	
// 	public static final boolean kEnableDashBoard = true;
	
//     // private DesiredMode mCachedDesiredMode = null;
//     // private StartingPosition mCachedStartingPosition = null;

    

//     static final double kShortInterval = .02;
//     static final double kLongInterval = .5;
    
//     static double m_shortOldTime = 0.0;
//     static double m_longOldTime = 0.0;   

//     /**
//      * Class Constructor
//      * Initialize the Dashboard with defaul values of "settable" inputs.
//      */
//     public DashBoard() {
//         if(kEnableDashBoard){
//             //SmartDashboard.putBoolean("Compressor ENABLE", true);
//             //SmartDashboard.putBoolean("Limelight-LED Toggle", false);

//             initStartupPostion();
//             initAutonomousStrategy();

//         }
//         m_thread.startPeriodic(kShortInterval);       
//     }

//     SendableChooser<StartPosition> m_startPositionChooser = new SendableChooser<>();

//     /** 
//      * Initialize Robot Starting Position
//      */
//     private void initStartupPostion(){
       
//         m_startPositionChooser.setDefaultOption("Top Tarmac, Up", StartPosition.TARMAC_1);
//         m_startPositionChooser.addOption("Top Tarmac, Low", StartPosition.TARMAC_2);
//         m_startPositionChooser.addOption("Bottom Tarmac, Left", StartPosition.TARMAC_3);
//         m_startPositionChooser.addOption("Bottom Tarmac, Right", StartPosition.TARMAC_4);

//         SmartDashboard.putData("Starting Position", m_startPositionChooser);

        

//     }


//     /** 
//      * Holds Robot Start Position
//      */
//     public StartPosition startPosition() {
//         return m_startPositionChooser.getSelected();
//       }



//     SendableChooser<Command> m_autonomousModeChooser = new SendableChooser<>();

//     /** 
//      * Initialize Robot Autonomous Strategy 
//      */
//     private void initAutonomousStrategy(){
        
        
//         m_autonomousModeChooser.setDefaultOption("Go to Terminal", terminalTrajectory);
//         m_autonomousModeChooser.setDefaultOption("Example", exampleTrajectory);
        
        
//         SmartDashboard.putData("Auto Mode", m_autonomousModeChooser);
        
//         SmartDashboard.putNumber("Some Number", 1.45);

        
//           }

         
          
    
       

 
//     public static void updateDashboard() {
//         double time = Timer.getFPGATimestamp();
//         if (kEnableDashBoard) {
//             if ((time - m_shortOldTime) > kShortInterval) {
//                 m_shortOldTime = time;
//                 updateShortInterval();
//             }
//             if ((time - m_longOldTime) > kLongInterval) {
//                 //Thing that should be updated every LONG_DELAY
//                 m_longOldTime = time;
//                 updateShortInterval();
//                 updateLongInterval();
//             }
//         }
//     }

//     /**
//      * Update values that need high frequency refresh.
//      **/   
//     private static void updateShortInterval() {
//         // RobotContainer.swerve.dashboard();
//         // RobotContainer.launcher.dashboard();
//         // RobotContainer.tower.dashboard();
//         // RobotContainer.intake.dashboard();
//         // RobotContainer.indexer.dashboard();
//         // RobotContainer.climber.dashboard();
//         // SmartDashboard.putBoolean("Pressure SW",
//         //         RobotContainer.compressor.getPressureSwitchValue());
//     }

//     /**
//      * Update values that need low frequency refresh.
//      **/
//     private static void updateLongInterval(){
//         //SmartDashboard.putBoolean("Compressor On?", RobotContainer.pneumatics.compressor.enabled());
        
//         //Can change to show a different message than "Yes" and "No"
//         // SmartDashboard.putBoolean("Change Battery", Util.changeBattery());
//     }


//     // static int t = 0;
//     // static boolean b = true;
//     // 
//     // public static void dashboardFlash(){
//     //     //Flash a light on the dashboard so that you know that the dashboard is refreshing.
//     //     if (t > 20) {
//     //         t = 0;
//     //         b = !b;
//     //         SmartDashboard.putBoolean("Disabled Toggle", b);
//     //     }
//     //     t++;
//     // }

//     private static class dashboardThread implements Runnable {
        
//         @Override
//         public void run() {
//             updateDashboard();
//         }
//     }

//       /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getStartCommand() {
//     Command startCommand = null;
//     startPosition = m_startPositionChooser.getselected;
//      autoStrat = m_autonomousModeChooser.getselected;
     
//      if (startPosition == TARMAC_1) {
     
     
//      switch (autoStrat){
//          case : startCommand = executeTrajectory1    
//                  BREAK;
//          case 2: startCommand = executeTrajectory2      
//                  BREAK;
//          case 3: startCommand = executeTrajectory3    
//                  BREAK;
//          default: null;
    

//     return startCommand;
//   }
  
//      }
//      else {
//         if (startPosition == TARMAC_2) {
//             }
//         } 
//     }
}
