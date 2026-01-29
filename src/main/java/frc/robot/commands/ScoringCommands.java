package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;

public class ScoringCommands
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


    public static void createCommands(RobotContainer robotContainer)
    {        
        System.out.println("  Constructor Started:  " + fullClassName);


        intake = robotContainer.getIntake();
        agitator = robotContainer.getAgitator();
        indexer = robotContainer.getIndexer();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        drivetrain = robotContainer.getDrivetrain();

        System.out.println("  Constructor Finished: " + fullClassName);

    }

    public static Command IntakeAndScoreCommand(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.pickupFuelCommand()),
                (agitator.forwardCommand()),
                (indexer.onCommand()),
                (accelerator.feedToShooterCommand(()-> 0.25)),
                (flywheel.shootCommand(() -> 10.0)));
        }
        else
        {
            return Commands.none();
        }
    } 

    public static Command IntakeAndScoreCommandTheSequal(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel( (accelerator.feedToShooterCommand(() -> 0.25)),
                    (flywheel.shootCommand(() -> 75.7))).until(flywheel.isAtSetSpeed(100, 5))
                    .andThen
                    // .commands.parallel(
                (intake.pickupFuelCommand()).andThen
                (agitator.forwardCommand()).andThen
                (indexer.onCommand());

        }
        else
        {
            return Commands.none();
        }
    } 


    public static Command StopIntakeAndScoreCommand(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.stopCommand()),
                (agitator.stopCommand()),
                (indexer.stopCommand()),
                (accelerator.stopCommand()),
                (flywheel.stopCommand()));
        }
        else
        {
            return Commands.none();
        }
    }

    // TODO implement with shooter speeds
    public static Command shootFromStandstillCommand(Drivetrain drivetrain, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && agitator != null && indexer != null && accelerator != null && flywheel != null)
        {
            return
            drivetrain.lockWheelsCommand()
            .andThen(
                drivetrain.angleLockDriveCommand(null, null, null, poseEstimator.getAngleToAllianceHub()).withTimeout(0.5))
            .andThen(
                flywheel.shootCommand(() -> flywheel.getShotPower(poseEstimator.getDistanceToAllianceHub().getAsDouble()))
                    .until(flywheel.isAtSetSpeed(flywheel.getShotPower(poseEstimator.getDistanceToAllianceHub().getAsDouble()), 2))) // within 2 feet per second
            .andThen(
                Commands.parallel(
                    agitator.forwardCommand(),
                    indexer.setForwardCommand(() -> 0.25),
                    accelerator.feedToShooterCommand(() -> 0.25)));
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * shoots fuel with constantly updating power values for flywheel while moving
     * @param drivetrain
     * @param agitator
     * @param indexer
     * @param accelerator
     * @param flywheel
     * @param poseEstimator
     * @return
     */
    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && agitator != null && indexer != null && accelerator != null && flywheel != null && poseEstimator != null)
        {
            Pose2d robotPose = drivetrain.getState().Pose;
            ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getRobotRelativeSpeeds(), robotPose.getRotation());

            Pose2d calculatedTarget = poseEstimator.getCalculatedTargetPose(
                poseEstimator.getAllianceHubPose(), 
                robotPose, 
                velocity);

            double distance = poseEstimator.getDistanceToTarget(robotPose, calculatedTarget).getAsDouble();
            double shooterPower = flywheel.getShotPower(distance);

            return
            flywheel.shootCommand(() -> shooterPower).until(flywheel.isAtSetSpeed(distance, 5))     // TODO tune tolerance
            .andThen(
                Commands.parallel(
                    accelerator.feedToShooterCommand(() -> 0.25),
                    indexer.setForwardCommand(() -> 0.25),
                    agitator.forwardCommand()));
        }
        else
        {
            return Commands.none();
        }
    }

    // public static Command ShootOnFly(Drivetrain drivetrain, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    // {

    // }
}
