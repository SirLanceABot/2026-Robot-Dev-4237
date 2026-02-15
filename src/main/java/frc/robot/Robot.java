// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandsManager;
import frc.robot.commands.StartUpCommands;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;
import frc.robot.pathplanner.PathPlannerLance;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final RobotContainer robotContainer;
    private Command autonomousCommand = Commands.none();
    private boolean isPreMatch = true;
    private TestMode testMode = null;
    

    private Command selectedCommand = Commands.none(); 
    private Command path = Commands.none();
    private String autoName = "Left2Cycle";

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    Robot() 
    {
        // The order of the code below is important.

        // 1. Configure the Data Loggers
        // DataLogFile.config();
        // new CommandSchedulerLog(
        //     EnumSet.of(CommandStageSelector.initialize, CommandStageSelector.interrupt, CommandStageSelector.finish, CommandStageSelector.execute), 
        //     EnumSet.of(LogsSelector.useConsole, LogsSelector.useDataLog, LogsSelector.useShuffleBoardLog));


        // 2. Create the RobotContainer
        robotContainer = new RobotContainer();

        // leds = robotContainer.getLEDs();

        // 3. Create the Commands
        CommandsManager.createCommands(robotContainer);


        // 4. Create the Trigger Bindings
        DriverBindings.createBindings(robotContainer);
        OperatorBindings.createBindings(robotContainer);

        // 5. Configure PathPlanner
        PathPlannerLance.configPathPlanner(robotContainer);
        // FollowPathCommand.warmupCommand().schedule();
        // PathfindingCommand.warmupCommand().schedule();

        // start the startup monitor (checks swerve alignment and controls LEDs)
        StartUpCommands.enableMonitor(robotContainer);
        // if(robotContainer.getDrivetrain() != null && robotContainer.getLEDs() != null)
        // {
        //     startupNotifier = new Notifier(StartUpCommands::checkAndUpdate);
        //     startupNotifier.startPeriodic(0.5);
        // }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() 
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() 
    {
        // Put code to run here before the match starts, but not between auto and teleop
        if(isPreMatch)
        {
            StartUpCommands.enableMonitor(robotContainer);
            // StartUpNotifier.startPeriodic(0.5);

            // autonomousCommand = PathPlannerLance.getAutonomousCommand();
            // path = PathPlannerLance.buildAutoPath(); // this made stuff not work.  may need to bring back in later if it works less good than not good

            autonomousCommand = PathPlannerLance.getAutonomousCommand();
            autoName = autonomousCommand.getName();

            if(AutoBuilder.isConfigured())
            {
                if(!autoName.equalsIgnoreCase("InstantCommand"))
                {
                    path = AutoBuilder.buildAuto(autoName);
                    PathPlannerLance.initializePose(autoName);
                }
            }
        }
    }

    /** This function is called periodically during Disabled mode. */
    @Override
    public void disabledPeriodic() 
    {
        // Put code to run here before the match starts, but not between auto and teleop
        if(isPreMatch)
        {
            selectedCommand = PathPlannerLance.getAutonomousCommand();
            if(!selectedCommand.getName().equalsIgnoreCase(autonomousCommand.getName()))
            {
                autonomousCommand = selectedCommand;
                autoName = autonomousCommand.getName();
                
                if(AutoBuilder.isConfigured())
                {
                    if(!autoName.equalsIgnoreCase("InstantCommand"))
                    {
                        path = AutoBuilder.buildAuto(autoName);
                        PathPlannerLance.initializePose(autoName);
                    }
                }
            }

            // moved to StartUpCommands
            // if(leds != null)
            // {
            //     if(!robotContainer.useFullRobot())
            //     {
            //         leds.setColorSolidCommand(100, Color.kYellow);
            //     }
            //     else if(selectedCommand != null)
            //     {
            //         leds.setColorSolidCommand(100, Color.kGreen);
            //     }
            //     else
            //     {
            //         leds.setColorSolidCommand(100, Color.kYellow);
            //     }
            // }
        }
    }

    /** This function is called once each time the robot exits Disabled mode. */
    @Override
    public void disabledExit() 
    {
        // if prematch = end notifier
        StartUpCommands.disableMonitor();
        // if (startupNotifier != null)
        // {
        //     startupNotifier.stop();
        // }
    }

    // public void initializePose()
    // {
    //     try {
    //         PathPlannerPath ppPath = PathPlannerPath.fromPathFile(autoName);
    //         Pose2d initialPose = ppPath.getStartingHolonomicPose().orElse(new Pose2d());
    //         robotContainer.getPoseEstimator().resetPose(initialPose);
    //     }
    //     catch (Exception e)
    //     {
    //         System.out.println("Path planner loading file error");
    //     }
    // }


    /** This function is called once each time the robot enters Autonomous mode. */
    @Override
    public void autonomousInit() 
    {
        // DataLogManager.start();

        isPreMatch = false;

        PathPlannerLance.initializePose(autonomousCommand.getName());

        if(path != null)
        {
            path.schedule();
            System.out.println("Scheduled Auto Command");
        }
    }

    /** This function is called periodically during Autonomous mode. */
    @Override
    public void autonomousPeriodic() 
    {}

    /** This function is called once each time the robot exits Autonomous mode. */
    @Override
    public void autonomousExit() 
    {
        // This makes sure that the autonomous stops running when teleop starts running.
        if (autonomousCommand != null) 
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }

        isPreMatch = false;
    }

    /** This function is called once each time the robot enters Teleop mode. */
    @Override
    public void teleopInit() 
    {
        // DataLogManager.start();

        // This makes sure that the autonomous stops running when teleop starts running.
        if (autonomousCommand != null) 
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }

        isPreMatch = false; 
    }

    /** This function is called periodically during Teleop mode. */
    @Override
    public void teleopPeriodic() 
    {}

    /** This function is called once each time the robot exits Teleop mode. */
    @Override
    public void teleopExit() 
    {
        // MotorControllerLance.logAllStickyFaults();
        // DataLogManager.stop();

        // isPreMatch = true;
    }

    /** This function is called once each time the robot enters Test mode. */
    @Override
    public void testInit() 
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        
        // Create a TestMode object to test one team members code.
        testMode = new TestMode(robotContainer);

        testMode.init();
    }

    /** This function is called periodically during Test mode. */
    @Override
    public void testPeriodic() 
    {
        testMode.periodic();
    }

    /** This function is called once each time the robot exits Test mode. */
    @Override
    public void testExit() 
    {
        testMode.exit();

        // Set the TestMode object to null so that garbage collection will remove the object.
        testMode = null;
    }
}
