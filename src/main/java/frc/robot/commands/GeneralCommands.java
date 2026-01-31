package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.PoseEstimator;

public class GeneralCommands
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

  // *** CLASS VARIABLES & INSTANCE VARIABLES ***
  // Put all class variables and instance variables here
    private static Intake intake; 
    private static Agitator agitator;
    private static Indexer indexer;
    private static Accelerator accelerator;
    private static Flywheel flywheel;
    private static Drivetrain drivetrain;
    private static PoseEstimator poseEstimator;
    private static Climb climb;


    public static void createCommands(RobotContainer robotContainer)
    {        
        System.out.println("  Constructor Started:  " + fullClassName);

        intake = robotContainer.getIntake();
        agitator = robotContainer.getAgitator();
        indexer = robotContainer.getIndexer();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        drivetrain = robotContainer.getDrivetrain();
        climb = robotContainer.getClimb();
        poseEstimator = robotContainer.getPoseEstimator();

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    // not tested
    public static Command intakeCommand(Intake intake)
    {
        if(intake != null)
        {
            return 
            Commands.parallel(
                intake.pickupFuelCommand(),
                agitator.forwardCommand(() -> 500.0) // rpm
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    public static Command resetIntakeCommand(Intake intake, Agitator agitator)
    {
        if(intake != null && agitator != null)
        {
            return 
            Commands.parallel(
                intake.retractIntakeCommand(),
                agitator.stopCommand()
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    public static Command ejectFuelInIntakeCommand(Intake intake, Agitator agitator)
    {
        if(intake != null && agitator != null)
        {
            return intake.ejectFuelCommand();
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    // ejects fuel backward relative to the shooter
    public static Command stopEjectingFuelInIntakeCommand(Intake intake, Agitator agitator)
    {
        if(intake != null && agitator != null)
        {
            return intake.retractIntakeCommand();
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    public static Command ejectAllFuelSlowlyCommand(Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {
        if(agitator != null && indexer != null && accelerator != null && flywheel != null)
        {
            return 
            flywheel.burpFuelCommand().until(flywheel.isAtSetSpeed(10.0, 1.0)) //velocity that can slowy eject fuel
            .andThen(
                Commands.parallel(
                    indexer.setForwardCommand(() -> 0.2),
                    accelerator.feedToShooterCommand(() -> 0.2),
                    agitator.forwardCommand(() -> 500.0) // rpm
                )
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    // ejects fuel forward relative to the shooter
    public static Command stopEjectingAllFuelCommand(Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {
        if(agitator != null && indexer != null && accelerator != null && flywheel != null)
        {
            return
            Commands.parallel(
                flywheel.stopCommand(),
                indexer.stopCommand(),
                accelerator.stopCommand(),
                agitator.stopCommand()
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    // no clue what actual climb will look like
    public static Command ascendToL1Command(Climb climb)
    {
        if(climb != null)
        {
            return Commands.run(() -> climb.ascendL1Command());
        }
        else
        {
            return Commands.none();
        }
    }

    // also not tested
    // still no clue what actual climb will look like
    public static Command descendFromL1Command(Climb climb, Drivetrain drivetrain)
    {
        if(climb != null)
        {
            return Commands.run(() -> climb.descendL1Command());
        }
        else
        {
            return Commands.none();
        }
    }

    // maybe L3?
}
