public class Autonomous{

    private final DashBoard m_dashboard;
    private final DriveSubsystem m_drivetrain;

    public Autonomous(DashBoard dashboard, DriveSubsystem drivetrain){
        m_dashboard = dashboard;
        m_drivetrain = drivetrain;
    }

    public Command getStartCommand() {
        Command startCommand = null;
        startPosition = dashboard.getStartPosition(); 
        autoStrat = dashboard.getAutoMode();
         
         switch (autoStrat) {
             
            case EXIT: System.out.println("Exit (Universal)")
            startCommand = new FollowTrajectory(Exit.json, m_drivetrain);
            public boolean isFollowFinished = startCommand.isFinished();
            
            break;
             
            case EXAMPLE: System.out.println("Running Test Trajectory)")
                startCommand = new TestTrajectory(m_drivetrain);
                public boolean isTestFinished = startCommand.isFinished();
                
                break;

            case GOAL_FIRST:
                switch (startPosition) { 
                    case TARMAC_1: 
                        System.out.println("GOAL FIRST from Tarmac 1")
                        // return new  Sequential Command Group: new FollowTrajectory(mytrajectory.json, m_drivetrain)()   ////./, new Shoot, new Intake on+ new Fiolklow(Traj1_2, drivetrain)
                        break;
    
                        case TARMAC_2: 
                        System.out.println("GOAL FIRST from Tarmac 2")
                        break;
    
                        case TARMAC_3: 
                        System.out.println("GOAL FIRST from Tarmac 3")
                        break;
                }
    
            case COLLECT_FIRST:
                switch (startPosition) { 
                    case TARMAC_1: 
                        System.out.println("COLLECT FIRST from Tarmac 1")
                        break;

                        case TARMAC_2: 
                        System.out.println("COLLECT FIRST from Tarmac 2")
                        break;

                        case TARMAC_3: 
                        System.out.println("COLLECT FIRST from Tarmac 3")
                        break;
                }
         }
        return startCommand;
      }
      
}