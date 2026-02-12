package frc.robot.commands;

import java.lang.invoke.MethodHandles;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.ColorPattern;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Climb.climbPosition;

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
    // private static Agitator agitator;
    private static Indexigator indexigator;
    private static Accelerator accelerator;
    private static Flywheel flywheel;
    private static Drivetrain drivetrain;
    private static PoseEstimator poseEstimator;
    private static Climb climb;
    private static LEDs leds;


    public static void createCommands(RobotContainer robotContainer)
    {        
        System.out.println("  Constructor Started:  " + fullClassName);

        intake = robotContainer.getIntake();
        // agitator = robotContainer.getAgitator();
        indexigator = robotContainer.getIndexigator();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        drivetrain = robotContainer.getDrivetrain();
        climb = robotContainer.getClimb();
        poseEstimator = robotContainer.getPoseEstimator();
        leds = robotContainer.getLEDs();

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    /**
     * Command to set the led color and pattern,
     * use this so that leds don't break the robot when disabled
     * @param pattern pattern of the led color(s)
     * @param colors the color(s) of the led
     * @return the command to set the led color and pattern
     * @author Matthew Fontecchio
     */
    public static Command setLEDCommand(ColorPattern pattern, Color... colors)
    {
        if(leds != null)
        {
            switch(pattern)
            {
                case kSolid:
                    return colors != null ? leds.setColorSolidCommand(100, colors[0]) : Commands.none();
                case kBlink:
                    return colors != null ? leds.setColorBlinkCommand(colors) : Commands.none();
                case kGradient:
                    return colors != null ? leds.setColorGradientCommand(100, colors) : Commands.none();
                case kRainbow:
                    return leds.setColorRainbowCommand();
                case kOff:
                    return leds.offCommand();
                default:
                    return Commands.none();
            }
        }
        else
        {
            return Commands.none();
        }
    }

    // color and pattern for LEDS to default to during a match
    public static Command defaultLEDCommand()
    {
        return setLEDCommand(ColorPattern.kSolid, Color.kRed).withName("Set LED to default (red)");
    }

    // tested
    /**
     * @author Robbie F
     * @return command to do the intake thing
     */
    public static Command intakeCommand()
    {
        if(intake != null && indexigator != null)
        {
            return 
            Commands.parallel(
                setLEDCommand(ColorPattern.kSolid, Color.kYellow),
                intake.pickupFuelCommand(),
                indexigator.setForwardCommand()
            .withName("Intaking Fuel"));
        }
        else
        {
            return Commands.none();
        }
    }

    // idk if we need this
    // may be necessary if to keep the hopper extended while not intaking the intake pivot must remain out
    // public static Command stopIntakeCommand()
    // {
    //     if(intake != null && agitator != null)
    //     {
    //         return
    //         Commands.parallel(
    //             intake.stopCommand(),
    //             agitator.stopCommand())
    //         .withName("Intake and Agitator Rollers Stopped (Pivot Still Extended)");
    //     }
    //     else
    //     {
    //         return Commands.none();
    //     }
    // }

    
    // tested
    /**
     * @author Robbie Frank
     * @return command stop intake rollers and reset it inside the robot
     */
    public static Command resetIntakeCommand()
    {
        if(intake != null && indexigator != null)
        {
            return 
            Commands.parallel(
                intake.retractIntakeCommand(),
                indexigator.stopCommand())
            .andThen(setLEDCommand(ColorPattern.kBlink, Color.kYellow)).withTimeout(0.5)
            .andThen(defaultLEDCommand())
            .withName("Intake Reset Into Robot");
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return command to reverse intake rollers and eject fuel in the intake
     */
    public static Command ejectFuelInIntakeCommand()
    {
        if(intake != null && indexigator != null)
        {
            return Commands.parallel(
                setLEDCommand(ColorPattern.kSolid, Color.kOrange),
                intake.ejectFuelCommand())
                .withName("Ejecting Fuel In Intake");
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return command to stop ejecting the fuel in the intake,
     * retracts intake pivot and stops the rollers
     */
    public static Command stopEjectingFuelInIntakeCommand()
    {
        if(intake != null && indexigator != null)
        {
            return intake.retractIntakeCommand()
            .andThen(defaultLEDCommand())
            .withName("Stopping Ejecting Fuel In Intake");
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * @author Logan
     * @return the thing
     */
    public static Command stopShootingCommand()
    {
        if(flywheel != null && indexigator != null && accelerator != null)
        {
            return
            Commands.parallel(
                flywheel.stopCommand(),
                indexigator.stopCommand(),
                accelerator.stopCommand()
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return command to do the thing to eject fuel from the shooter
     */
    public static Command ejectAllFuelSlowlyCommand()
    {
        if(indexigator != null && accelerator != null && flywheel != null)
        {
            return 
            Commands.parallel(
            setLEDCommand(ColorPattern.kSolid, Color.kOrange),
            flywheel.burpFuelCommand().until(flywheel.isAtSetSpeed(10.0, 1.0))) // velocity that can slowy eject fuel
            .andThen(
                Commands.parallel(
                    indexigator.setForwardCommand(() -> 0.2),
                    accelerator.feedToShooterCommand(() -> 0.2)))
            .withName("Ejecting All Fuel Slowly");
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return commmand to stop ejecting fuel from the shooter
     */
    public static Command stopEjectingAllFuelCommand()
    {
        if(indexigator != null && accelerator != null && flywheel != null)
        {
            return
            Commands.parallel(
                flywheel.stopCommand(),
                indexigator.stopCommand(),
                accelerator.stopCommand())
            .andThen(defaultLEDCommand())
            .withName("Stopped Ejecting All Fuel");
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return extend climb so ready climb
     */
    public static Command extendClimbToL1Command()
    {
        if(climb != null)
        {
            return setLEDCommand(ColorPattern.kRainbow)
            .andThen(climb.extendToL1Command())
            .until(climb.isAtPosition(climbPosition.kEXTENDL1))
            .withName("Climb Extended To L1 Position");       
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * @author Robbie F
     * @return command to retract climb after locking in to L1
     */
    public static Command ascendFromL1Command()
    {
        if(climb != null)
        {
            return climb.retractFromL1Command()
            .until(climb.isAtPosition(climbPosition.kRETRACTL1))
            .withName("Climb Mounted L1");
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * @author Robbie F
     * @return yay climb
     */
    public static Command descendFromL1Command()
    {
        if(climb != null)
        {
            return climb.extendToL1Command()
            .until(climb.isAtPosition(climbPosition.kEXTENDL1))
            // .andThen( () -> drivetrain.resetForFieldCentric())
            .withName("Climb Unmounted L1");
        } 
        else
        {
            return Commands.none();
        }
    }
    
    /**
     * @author Robbie F
     * @return command to reset climb to the starting position
     */
    public static Command resetClimbToStartCommand()
    {
        if(climb != null)
        {
            return climb.resetToStartCommand()
            .until(climb.isAtPosition(climbPosition.kSTART))
            .withName("climb reset to inside robot");
        }
        else
        {
            return Commands.none();
        }
    }

    // not tested
    /**
     * @author Robbie F
     * @return command to climb to L1
     */
    public static Command climbToL1Command()
    {
        if(climb != null)
        {
            return extendClimbToL1Command()
            .andThen(ascendFromL1Command())
            .withName("did the climb thing");
        }
        else
        { 
            return Commands.none();
        }
    }

    //not tested
    /**
     * @author Robbie F
     * @return command to retract from L1
     */
    public static Command retractFromL1Command()
    {
        if(climb != null)
        {
            return descendFromL1Command()
            .andThen(resetClimbToStartCommand())
            .andThen(defaultLEDCommand())
            .withName("reset climbing mechanism from L1");
        }
        else
        {
            return Commands.none();
        }
    }
    
    /**
     * Drives autonomously from the given pose to the target pose
     * @param targetPose
     * @param currentPose
     * @return
     * @author Biggie Cheese
     */
    public static Command driveToPositionCommand(Pose2d targetPose, Pose2d currentPose)
    {
        PathConstraints constraints = new PathConstraints(0.5, 0.5, Units.degreesToRadians(360), Units.degreesToRadians(360));
        
        Rotation2d pathTangent = new Rotation2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY()
        );
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                                    new Pose2d(currentPose.getTranslation(), pathTangent),
                                    new Pose2d(targetPose.getTranslation(), pathTangent));          

        double vxMetersPerSecond = drivetrain.getState().Speeds.vxMetersPerSecond;
        double vyMetersPerSecond = drivetrain.getState().Speeds.vyMetersPerSecond;

        double velocity = Math.sqrt(vxMetersPerSecond * vxMetersPerSecond + vyMetersPerSecond * vyMetersPerSecond);

        Rotation2d rotation = drivetrain.getPose().getRotation();

        IdealStartingState idealStartingState = new IdealStartingState(velocity, rotation);

        PathPlannerPath path = new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    idealStartingState, // set this to null if not working
                                    new GoalEndState(0.0, targetPose.getRotation()));
        path.preventFlipping = true;


        return AutoBuilder.followPath(path);
    }

    
    // maybe L3?

    // ask Grant






    
















    // please?
}
