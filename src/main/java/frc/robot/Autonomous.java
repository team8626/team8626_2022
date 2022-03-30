// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Team8626 Libraries
import frc.robot.DashBoard.AutoSelec;
import frc.robot.DashBoard.StartPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.TestTrajectoryCommand;

public class Autonomous {

    private final DashBoard m_dashboard;
    private final DriveSubsystem m_drivetrain;

    public StartPosition m_startPosition;
    public AutoSelec m_autoStart;

    public Autonomous(DashBoard dashboard, DriveSubsystem drivetrain){
        m_dashboard = dashboard;
        m_drivetrain = drivetrain;
    }

    public Command getStartCommand() throws IOException {
        Command startCommand = null;
        m_startPosition = m_dashboard.getStartPosition(); 
        m_autoStart = m_dashboard.getAutoMode();
         
        switch (m_autoStart) {
            default: 
            case EXIT: 
                System.out.println("Exit (Universal)");
                startCommand = new FollowTrajectoryCommand("Exit.json", m_drivetrain);
                break;
             
            case TEST: System.out.println("Running Test Trajectory)");
                startCommand = new TestTrajectoryCommand(m_drivetrain);            
                break;

            case EXAMPLE: 
                startCommand = 
                    new SequentialCommandGroup(
                        new FollowTrajectoryCommand("example.json", m_drivetrain),
                        new WaitCommand(5)); 

            case GOAL_FIRST:
                switch (m_startPosition) { 
                    case TARMAC_1: 
                        System.out.println("GOAL FIRST from Tarmac 1");
                        // return new  Sequential Command Group: new FollowTrajectoryCommand(mytrajectory.json, m_drivetrain)()   ////./, new Shoot, new Intake on+ new Fiolklow(Traj1_2, drivetrain)
                        break;
    
                    case TARMAC_2: 
                        System.out.println("GOAL FIRST from Tarmac 2");
                        break;

                    case TARMAC_3: 
                        System.out.println("GOAL FIRST from Tarmac 3");
                        break;
                }
    
            case COLLECT_FIRST:
                switch (m_startPosition) { 
                    case TARMAC_1: 
                        System.out.println("COLLECT FIRST from Tarmac 1");
                        break;

                    case TARMAC_2: 
                        System.out.println("COLLECT FIRST from Tarmac 2");
                        break;

                    case TARMAC_3: 
                        System.out.println("COLLECT FIRST from Tarmac 3");
                        break;
                }
         }
        return startCommand;
    }  
}