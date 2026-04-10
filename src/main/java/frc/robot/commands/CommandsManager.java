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
import frc.robot.subsystems.Climb;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExpandingHopper;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.PoseEstimator;

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
    // private static Agitator agitator;
    private static Accelerator accelerator;
    private static Flywheel flywheel;
    private static PoseEstimator poseEstimator;
    private static Indexigator indexigator;
    private static Climb climb;
    private static ExpandingHopper expandingHopper;

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
        // agitator = robotContainer.getAgitator();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        poseEstimator = robotContainer.getPoseEstimator();
        indexigator = robotContainer.getIndexigator();
        climb = robotContainer.getClimb();
        expandingHopper = robotContainer.getExpandingHopper();


        createNamedCommands();

        System.out.println("Constructor Finsihed: " + fullClassName);
    }

    private static void createNamedCommands()
    {
        // Intaking Commands
        NamedCommands.registerCommand("Intake Command", GeneralCommands.intakeCommand());
        NamedCommands.registerCommand("Reset Intake Command", GeneralCommands.resetIntakeCommand());
        NamedCommands.registerCommand("Stop Intaking Command", GeneralCommands.stopIntakingCommand());
        NamedCommands.registerCommand("Move Intake Out Command", GeneralCommands.moveIntakeOutCommand());

        if(expandingHopper != null)
        {
            NamedCommands.registerCommand("Expand Hopper Command", expandingHopper.extendHopperCommand());
            NamedCommands.registerCommand("Retract Hopper Command", expandingHopper.retractHopperCommand());
        }

        // Ejecting Commands
        NamedCommands.registerCommand("Eject Fuel From Intake Command", GeneralCommands.ejectFuelInIntakeCommand());
        NamedCommands.registerCommand("Stop Ejecting Fuel From Intake Command", GeneralCommands.stopEjectingFuelInIntakeCommand());
        NamedCommands.registerCommand("Eject All Fuel Slowly Command", GeneralCommands.ejectAllFuelSlowlyCommand());
        NamedCommands.registerCommand("Stop Ejecting All Fuel Slowly Command", GeneralCommands.stopEjectingAllFuelCommand());
        NamedCommands.registerCommand("Stop Shooting Command", GeneralCommands.stopShootingCommand());
        NamedCommands.registerCommand("Unjam Intake Command", GeneralCommands.unjamIntakeCommand());
        NamedCommands.registerCommand("Agitate Intake Command", GeneralCommands.agitateIntakeCommand().repeatedly());

        // Scoring Commands
        NamedCommands.registerCommand("Shoot From Standstill Command", ScoringCommands.shootFromStandstillCommand(drivetrain, indexigator, accelerator, flywheel, poseEstimator));
        NamedCommands.registerCommand("Shoot on the Move", ScoringCommands.shootOnTheMoveCommand(drivetrain, indexigator, accelerator, flywheel, poseEstimator).repeatedly());
        NamedCommands.registerCommand("Physics Shoot on the Move", ScoringCommands.physicsShootOnTheMove(drivetrain, poseEstimator, indexigator, accelerator, flywheel));
        NamedCommands.registerCommand("Pass Command", ScoringCommands.passCommand(indexigator, accelerator, flywheel));
        NamedCommands.registerCommand("Ramp up Flywheel Command", GeneralCommands.getFlywheelToSpeedCommand());
        // NamedCommands.registerCommand("Angle Lock", drivetrain.angleLockDriveCommand(null, null, null, () -> (poseEstimator.getRotationToCalculatedTarget(poseEstimator.getAllianceHubPose()).getAsDouble())));

        // Climbing Commands
        NamedCommands.registerCommand("Extend Climb To L1 Command", GeneralCommands.extendClimbToL1Command());
        NamedCommands.registerCommand("Ascend From L1 Command", GeneralCommands.ascendFromL1Command());
        NamedCommands.registerCommand("Left Auto Climb L1 Command", ScoringCommands.autoClimbCommand(drivetrain, poseEstimator, climb, ()-> true));
        NamedCommands.registerCommand("Right Auto Climb L1 Command", ScoringCommands.autoClimbCommand(drivetrain, poseEstimator, climb, ()-> false));

        
        SmartDashboard.putData(CommandScheduler.getInstance());
    }
}
