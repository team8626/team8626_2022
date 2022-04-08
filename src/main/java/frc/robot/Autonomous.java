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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.commands.Autonomous_CollectAndShootTwoCommand;
import frc.robot.commands.Autonomous_NewShootAndMoveCommand;
import frc.robot.commands.DriveMetersCommand;
import frc.robot.commands.DriveStraightMetersCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.Autonomous_ShootAndMoveCommand;
import frc.robot.commands.Autonomous_ShootAndMoveMetersCommand;
import frc.robot.commands.TestTrajectoryCommand;
import frc.robot.commands.TurnDegreesCommand;

public class Autonomous {

    private final DashBoard m_dashboard;
    private final DriveSubsystem m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final StorageSubsystem m_storage;
    private final ShooterSubsystem m_shooter;

    public StartPosition m_startPosition;
    public AutoSelec m_autoStart;

    public Autonomous(DashBoard dashboard, DriveSubsystem drivetrain, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter){
        m_dashboard = dashboard;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_storage = storage;
        m_shooter = shooter;
    }

    public Command getStartCommand() throws IOException {
        Command startCommand = null;
        m_startPosition = m_dashboard.getStartPosition(); 
        m_autoStart = m_dashboard.getAutoMode();
         
        switch (m_autoStart) {
            default: 
            case SHOOT_AND_MOVE:
                // Shoot then move out of the tarmac
                //startCommand = new Autonomous_ShootAndMoveCommand(m_drivetrain, m_storage, m_shooter);
                startCommand = new Autonomous_NewShootAndMoveCommand(m_drivetrain, m_storage, m_shooter);
                break;

            case SHOOT_AND_MOVE_METERS:
                 // Shoot then move out of the tarmac
                 startCommand = new Autonomous_ShootAndMoveMetersCommand(-2.15 /* distance in m */, m_drivetrain, m_storage, m_shooter);
                break;

            case COLLECT_AND_SHOOT2:
                // Collect one extra ball then shoot 2 balls
                startCommand = new Autonomous_CollectAndShootTwoCommand(3 /* distance in m */, m_drivetrain, m_intake, m_storage, m_shooter);
                break;
            
            case EXIT: 
                System.out.println("[AUTONOMOUS] Exit (Universal])");
                startCommand = new DriveMetersCommand(() -> 2.15 /* meters */ , () -> 0.75 /* Power */, m_drivetrain);
                //startCommand = new FollowTrajectoryCommand("Exit.json", m_drivetrain);
                break;
             
            case TEST: 
                System.out.println("[AUTONOMOUS] Drive 1 Meter");
                startCommand = new DriveMetersCommand(() -> 1 /* meters */ , () -> 0.5 /* Power */, m_drivetrain);
                    break;
    
            case TEST_1M: 
                System.out.println("[AUTONOMOUS] Drive 1 Meter");
                startCommand = new DriveStraightMetersCommand(() -> 1 /* meters */ , () -> 0.5 /* Power */, m_drivetrain);
                break;
    
            case TEST_1M_STRAIGHT: 
                System.out.println("[AUTONOMOUS] Turn 180 Degrees");
                startCommand = new TestTrajectoryCommand(m_drivetrain);            
                break;

            case TEST_TURN180: 
                System.out.println("Running Test Trajectory)");
                startCommand = new TurnDegreesCommand(() -> 180 /* degrees */ , () -> 0.5 /* Power */, m_drivetrain);
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