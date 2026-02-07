// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PoseEstimator;

/** 
 * An example command that uses an example subsystem. 
 */
public class CommandsManager extends Command 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS AND INSTANCE VARIABLES ***
    private static Drivetrain drivetrain;
    private static Agitator agitator;
    private static Accelerator accelerator;
    private static Flywheel flywheel;
    private static PoseEstimator poseEstimator;
    private static Indexer indexer;

    /**
     * Creates a new Commands Manager
     */
    public CommandsManager() 
    {}

    public static void createCommands(RobotContainer robotContainer)
    {
        System.out.println("Constructor Started: " + fullClassName);

        GeneralCommands.createCommands(robotContainer);
        ScoringCommands.createCommands(robotContainer);

        drivetrain = robotContainer.getDrivetrain();
        agitator = robotContainer.getAgitator();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        poseEstimator = robotContainer.getPoseEstimator();
        indexer = robotContainer.getIndexer();


        createNamedCommands();

        System.out.println("Constructor Finsihed: " + fullClassName);
    }

    private static void createNamedCommands()
    {
        // Intaking Commands
        NamedCommands.registerCommand("Intake Command", GeneralCommands.intakeCommand());
        NamedCommands.registerCommand("Reset Intake Command", GeneralCommands.resetIntakeCommand());

        // Ejecting Commands
        NamedCommands.registerCommand("Eject Fuel From Intake Command", GeneralCommands.ejectFuelInIntakeCommand());
        NamedCommands.registerCommand("Stop Ejecting Fuel From Intake Command", GeneralCommands.stopEjectingFuelInIntakeCommand());
        NamedCommands.registerCommand("Eject All Fuel Slowly Command", GeneralCommands.ejectAllFuelSlowlyCommand());
        NamedCommands.registerCommand("Stop Ejecting All Fuel Slowly Command", GeneralCommands.stopEjectingAllFuelCommand());

        // Scoring Commands
        NamedCommands.registerCommand("Shoot From Standstill Command", ScoringCommands.shootFromStandstillCommand(drivetrain, agitator, accelerator, flywheel, poseEstimator));
        NamedCommands.registerCommand("Shoot on the Move", ScoringCommands.shootOnTheMoveCommand(drivetrain, agitator, indexer, accelerator, flywheel, poseEstimator));
        NamedCommands.registerCommand("Physics Shoot on the Move", ScoringCommands.physicsShootOnTheMove(drivetrain, poseEstimator, agitator, indexer, accelerator, flywheel));
        NamedCommands.registerCommand("Pass Command", ScoringCommands.passCommand(agitator, accelerator, flywheel));

        // Climbing Commands
        NamedCommands.registerCommand("Climb to L1 Command", GeneralCommands.climbToL1Command());
        NamedCommands.registerCommand("Retract from L1 Command", GeneralCommands.retractFromL1Command());

        
        SmartDashboard.putData(CommandScheduler.getInstance());
    }
}
