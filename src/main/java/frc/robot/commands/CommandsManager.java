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
import frc.robot.subsystems.ExampleSubsystem;

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


    /**
     * Creates a new Commands Manager
     */
    public CommandsManager() 
    {}

    public static void createCommands(RobotContainer robotcontainer)
    {
        System.out.println("Constructor Started: " + fullClassName);

        GeneralCommands.createCommands(robotcontainer);
        ScoringCommands.createCommands(robotcontainer);

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

        // Climbing Commands
        NamedCommands.registerCommand("Extend Climb To L1 Command", GeneralCommands.extendClimbToL1Command());
        NamedCommands.registerCommand("Ascend L1 Command", GeneralCommands.ascendL1Command());
        NamedCommands.registerCommand("Descend L1 Command", GeneralCommands.descendFromL1Command());

        
        SmartDashboard.putData(CommandScheduler.getInstance());
    }
}
