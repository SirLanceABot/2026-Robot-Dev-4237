package frc.robot.commands;

import java.lang.invoke.MethodHandles;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
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
    // private static Agitator agitator;
    private static Indexigator indexigator;
    private static Accelerator accelerator;
    private static Flywheel flywheel;
    private static Drivetrain drivetrain;
    private static PoseEstimator poseEstimator;


    public static void createCommands(RobotContainer robotContainer)
    {        
        System.out.println("  Constructor Started:  " + fullClassName);

        intake = robotContainer.getIntake();
        // agitator = robotContainer.getAgitator();
        indexigator = robotContainer.getIndexigator();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        drivetrain = robotContainer.getDrivetrain();

        System.out.println("  Constructor Finished: " + fullClassName);

    }

    public static Command IntakeAndScoreCommand(Intake intake, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && indexigator != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.pickupFuelCommand()),
                (indexigator.onCommand()),
                (accelerator.feedToShooterCommand(()-> 0.25)),
                (flywheel.setControlVelocityCommand(() -> 10.0)));
        }
        else
        {
            return Commands.none();
        }
    } 

    public static Command IntakeAndScoreCommandTheSequal(Intake intake, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel)
    {        
        if(intake != null && indexigator != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel( (accelerator.feedToShooterCommand(() -> 0.25)),
                    (flywheel.setControlVelocityCommand(() -> 75.7))).until(flywheel.isAtSetSpeed(100, 5))
                    .andThen
                    // .commands.parallel(
                (intake.pickupFuelCommand()).andThen
                (indexigator.onCommand());

        }
        else
        {
            return Commands.none();
        }
    } 


    public static Command stopIntakeAndShooterCommand(Intake intake, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel)
    {
        if(intake != null  && indexigator != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.stopCommand()),
                (indexigator.stopCommand()),
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
    public static Command shootFromStandstillCommand(Drivetrain drivetrain, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && indexigator != null && accelerator != null && flywheel != null && poseEstimator != null)
        {
            DoubleSupplier distance = () -> (poseEstimator.getDistanceToTarget(drivetrain.getState().Pose, poseEstimator.getAllianceHubPose()).getAsDouble());
            DoubleSupplier shooterPower = () -> (flywheel.getShotPower(distance.getAsDouble() * 3.281));
            return
            drivetrain.lockWheelsCommand().withTimeout(0.1)
            .andThen(
                drivetrain.angleLockDriveCommand(() -> 0, () -> 0, () -> 0.05, () -> (poseEstimator.getAngleToAllianceHub().getAsDouble())).withTimeout(0.75))
            .andThen(
                flywheel.setControlVelocityCommand(() -> (shooterPower.getAsDouble()))) // meters -> feet
                    .until(() -> flywheel.isAtSetSpeed(shooterPower.getAsDouble(), 10).getAsBoolean()) // within 2 feet per second
            .andThen(
                Commands.parallel(
                    indexigator.setForwardCommand(), // rpm
                    accelerator.setVelocityCommand(12.0)));
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * shoots fuel with constantly updating power values for flywheel while moving
     * @param drivetrain
     * @param indexigator
     * @param accelerator
     * @param flywheel
     * @param poseEstimator
     * @return
     * @author Logan Bellinger
     */
    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel, PoseEstimator poseEstimator)
    {
        if(drivetrain != null && indexigator != null && accelerator != null && flywheel != null && poseEstimator != null)
        {
            Supplier<Pose2d> robotPose =  () -> drivetrain.getState().Pose;
            Supplier<ChassisSpeeds> velocity = () -> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getRobotRelativeSpeeds(), robotPose.get().getRotation());

            Supplier<Pose2d> calculatedTarget = () -> poseEstimator.getCalculatedTargetPose( // Pose of our caluclated target, adjusting for robot velo
                poseEstimator.getAllianceHubPose(), 
                robotPose.get(), 
                velocity.get());

            DoubleSupplier distance = () -> (poseEstimator.getDistanceToTarget(robotPose.get(), calculatedTarget.get()).getAsDouble());
            DoubleSupplier shooterPower = () -> (flywheel.getShotPower(distance.getAsDouble() * 3.281)); // meters -> feet

            return
            flywheel.setControlVelocityCommand(() -> shooterPower.getAsDouble()).until(flywheel.isAtSetSpeed(shooterPower.getAsDouble(), 10))     // TODO tune tolerance
            .andThen(
                Commands.parallel(
                    indexigator.setForwardCommand(), // rpm
                    accelerator.setVelocityCommand(12.0)));
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
     * @param indexigator
     * @param accelerator
     * @param flywheel
     * @return shooterVelocity to shoot on the move
     * @author Matthew
     */
    public static Command physicsShootOnTheMove(Drivetrain drivetrain, PoseEstimator poseEstimator, Indexigator indexigator, Accelerator accelerator, Flywheel flywheel)
    {
        if(drivetrain != null && poseEstimator != null && indexigator != null && accelerator != null && flywheel != null)
        {
            double shooterVelocity = poseEstimator.pureShooterVelocity(poseEstimator.getAllianceHubPose()).getAsDouble();

            return
            flywheel.setControlVelocityCommand(() -> shooterVelocity).until(flywheel.isAtSetSpeed(shooterVelocity, 5))   //TODO also tun this tolerance
            .andThen(
                Commands.parallel(
                    indexigator.setForwardCommand(),
                    accelerator.feedToShooterCommand(() -> 0.1)));
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command passCommand(Indexigator indexigator, Accelerator accelerator, Flywheel flywheel)
    {
        if(indexigator != null && accelerator != null && flywheel != null)
        {
            return
            flywheel.setControlVelocityCommand(() -> 60.0).until(flywheel.isAtSetSpeed(60.0, 10))   // test value
            .andThen(
            Commands.parallel(
                indexigator.setForwardCommand(), //rpm
                accelerator.setVelocityCommand(12.0)));
        }
        else
        {
            return Commands.none();
        }
        
    }

    // Have tested going to a set point with pathplanner on the fly and that works -> have not implemented climb or dynamic end poses
    public static Command autoClimbCommand(Drivetrain drivetrain, PoseEstimator poseEstimator, Climb climb, BooleanSupplier isLeft)
    {
        if(drivetrain != null && poseEstimator != null && climb != null)
        {
            Pose2d targetClimbPose;
            
            if(drivetrain.isRedAllianceSupplier().getAsBoolean())
            {
                if(isLeft.getAsBoolean())
                {
                    targetClimbPose = new Pose2d( new Translation2d(15.085, 3.437), new Rotation2d(180));
                }
                else
                {
                    targetClimbPose = new Pose2d( new Translation2d(15.906, 5.235), new Rotation2d(0));
                }
            }
            else
            {
                if(isLeft.getAsBoolean())
                {
                    targetClimbPose = new Pose2d( new Translation2d(1.482, 4.633), new Rotation2d(0));
                }
                else
                {
                    targetClimbPose = new Pose2d( new Translation2d(0.634, 2.835), new Rotation2d(180));
                }
            }

            // targetClimbPose = new Pose2d(4.847, 6.67, new Rotation2d());

            // need to check if intake is 
            if(!intake.isExtended().getAsBoolean())
            {
                return
                Commands.parallel(
                    GeneralCommands.extendClimbToL1Command(),
                    GeneralCommands.driveToPositionCommand(targetClimbPose, drivetrain.getState().Pose))
                .andThen(
                    GeneralCommands.ascendFromL1Command());
            }
            else
            {
                return Commands.none();
            }
        }
        else
        {
            return Commands.none();
        }
    }

    // NOT TESTED
    public static Command autoClimbDownCommand(Drivetrain drivetrain, PoseEstimator poseEstimator, Climb climb, BooleanSupplier isLeft)
    {
        if(drivetrain != null && poseEstimator != null && climb != null)
        {
            Pose2d targetPose;

            if(drivetrain.isRedAllianceSupplier().getAsBoolean())
            {
                if(isLeft.getAsBoolean())
                {
                    targetPose = new Pose2d( new Translation2d(4237.0, 4237.0), new Rotation2d(4237.0));
                }
                else
                {
                    targetPose = new Pose2d( new Translation2d(4237.0, 4237.0), new Rotation2d(4237.0));
                }
            }
            else
            {
                if(isLeft.getAsBoolean())
                {
                    targetPose = new Pose2d( new Translation2d(4237.0, 4237.0), new Rotation2d(4237.0));
                }
                else
                {
                    targetPose = new Pose2d( new Translation2d(4237.0, 4237.0), new Rotation2d(4237.0));
                }
            }

            return
            GeneralCommands.descendFromL1Command()
            .andThen( ()-> drivetrain.resetForFieldCentric())
            .andThen(GeneralCommands.driveToPositionCommand(targetPose, drivetrain.getState().Pose))
            .andThen(GeneralCommands.resetClimbToStartCommand());
        }
        else
        {
            return Commands.none();
        }
    }
}
