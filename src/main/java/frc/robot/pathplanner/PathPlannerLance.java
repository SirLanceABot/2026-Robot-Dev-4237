package frc.robot.pathplanner;

import java.lang.invoke.MethodHandles;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;

public class PathPlannerLance
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

    private static Drivetrain drivetrain;
    private static Field2d field;
    private static PoseEstimator poseEstimator;

    private static SendableChooser<Command> autoChooser;


    private PathPlannerLance()
    {}

    public static void configPathPlanner(RobotContainer robotContainer)
    {
        drivetrain = robotContainer.getDrivetrain();
        poseEstimator = robotContainer.getPoseEstimator();

        configAutoChooser();
        // getAutonomousCommand();
        // configAutoBuilder();

        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();

    }

    private static BooleanSupplier shouldFlipPath()
    {
        return 
        () -> 
        {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if(alliance.isPresent()) 
            {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }

    private static void configAutoChooser()
    {
        if(AutoBuilder.isConfigured())
        {
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
        else
        {
            autoChooser = new SendableChooser<Command>();
            autoChooser.setDefaultOption("None", Commands.none());
        }
    }

    public static Command getAutonomousCommand() 
    {
        if(autoChooser != null)
        {
            return autoChooser.getSelected();
        }
        else
        {
            return Commands.none();
        }
        // return new PathPlannerAuto("TEST - Move 2m");

    }

    public static void initializePose(String autoName)
    {
        try 
        {
            PathPlannerPath ppPath = PathPlannerPath.fromPathFile(autoName);
            Pose2d initialPose = ppPath.getStartingHolonomicPose().orElse(new Pose2d());
            poseEstimator.resetPose(initialPose);
        }
        catch (Exception e)
        {
            System.out.println("Path planner loading file error");
        }
    }

    public static Command buildAutoPath()
    {
        String autoName = getAutonomousCommand().getName();
        System.out.println("Auto name: " + autoName);

        if(AutoBuilder.isConfigured())
        {
            Command path = AutoBuilder.buildAuto(autoName);
            initializePose(autoName);

            return path;
        }
        else 
        {
            System.out.println("Invalid Auto Selection");
            return Commands.none();
        }
    }

     private static SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public static void configAutoBuilder()
    {
        try
        {
            RobotConfig config = RobotConfig.fromGUISettings();

            // Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;
            // Supplier<ChassisSpeeds> stateSupplier = () -> drivetrain.getState().Speeds;
            
            AutoBuilder.configure
            (
                () -> drivetrain.getState().Pose,
                (pose) -> drivetrain.resetPose(pose),
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)
                                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(9, 0, 0),
                    new PIDConstants(8.5, 0, 0) // used to be 7
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
        }
    }
}