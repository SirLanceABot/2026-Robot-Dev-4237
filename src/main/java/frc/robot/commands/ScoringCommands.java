package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
                (flywheel.setControlVelocityCommand(() -> 10.0)));
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
                    (flywheel.setControlVelocityCommand(() -> 75.7))).until(flywheel.isAtSetSpeed(100, 5))
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
    // alignment to hub works and flywheel/agitator/accelerator parts work seperately, test together with full robot
    // have NOT tested implementeation with varying powers/distances
    public static Command shootFromStandstillCommand(Drivetrain drivetrain, Agitator agitator, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && agitator != null  && accelerator != null && flywheel != null && poseEstimator != null)
        {
            return
            drivetrain.lockWheelsCommand().withTimeout(0.1)
            .andThen(
                drivetrain.angleLockDriveCommand(() -> 0, () -> 0, () -> 0.05, () -> (poseEstimator.getAngleToAllianceHub().getAsDouble())).withTimeout(0.75))
            .andThen(
                flywheel.setControlVelocityCommand(() -> (flywheel.getShotPower(poseEstimator.getDistanceToAllianceHub().getAsDouble() * 3.281))) // meters -> feet
                    .until(() -> flywheel.isAtSetSpeed(flywheel.getShotPower(poseEstimator.getDistanceToAllianceHub().getAsDouble() * 3.281), 5).getAsBoolean())) // within 2 feet per second
            .andThen(
                Commands.parallel(
                    agitator.forwardCommand(), // rpm
                    accelerator.feedToShooterCommand(() -> 0.1)));
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
     * @author Logan Bellinger
     */
    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && agitator != null && indexer != null && accelerator != null && flywheel != null && poseEstimator != null)
        {
            Pose2d robotPose = drivetrain.getState().Pose;
            ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getRobotRelativeSpeeds(), robotPose.getRotation());

            Pose2d calculatedTarget = poseEstimator.getCalculatedTargetPose( // Pose of our caluclated target, adjusting for robot velo
                poseEstimator.getAllianceHubPose(), 
                robotPose, 
                velocity);

            double distance = poseEstimator.getDistanceToTarget(robotPose, calculatedTarget).getAsDouble();
            double shooterPower = flywheel.getShotPower(distance * 3.281); // meters -> feet

            return
            flywheel.setControlVelocityCommand(() -> shooterPower).until(flywheel.isAtSetSpeed(shooterPower, 5))     // TODO tune tolerance
            .andThen(
                Commands.parallel(
                    agitator.forwardCommand(), // rpm
                    accelerator.feedToShooterCommand(() -> 0.1)));
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * 
     * Same as shootOnTheMoveCommand: shoots fuel with constantly updating power values for flywheel while moving,
     * BUT uses purely calculations to determine shooterVelocity
     * @param drivetrain
     * @param poseEstimator
     * @param agitator
     * @param indexer
     * @param accelerator
     * @param flywheel
     * @return shooterVelocity to shoot on the move
     * @author Matthew
     */
    public static Command physicsShootOnTheMove(Drivetrain drivetrain, PoseEstimator poseEstimator, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {
        if(drivetrain != null && poseEstimator != null && agitator != null && indexer != null && accelerator != null && flywheel != null)
        {
            double shooterVelocity = poseEstimator.pureShooterVelocity(poseEstimator.getAllianceHubPose()).getAsDouble();

            return
            flywheel.setControlVelocityCommand(() -> shooterVelocity).until(flywheel.isAtSetSpeed(shooterVelocity, 5))   //TODO also tun this tolerance
            .andThen(
                Commands.parallel(
                    agitator.forwardCommand(),
                    accelerator.feedToShooterCommand(() -> 0.1)));
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command passCommand(Agitator agitator, Accelerator accelerator, Flywheel flywheel)
    {
        return
        flywheel.setControlVelocityCommand(() -> 10.0).until(flywheel.isAtSetSpeed(10.0, 5))   // test value
        .andThen(
            Commands.parallel(
                (agitator.forwardCommand()), //rpm
                (accelerator.feedToShooterCommand(() -> 0.1))));
    }
}
