package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.sensors.Camera;

/**
 * This is an example of what a subsystem should look like.
 */
public class PoseEstimator extends SubsystemBase
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final Pigeon2 gyro;
    private final Drivetrain drivetrain;
    private final Camera[] cameraArray;

    private final SwerveDrivePoseEstimator poseEstimator;

    private Pose2d estimatedPose = new Pose2d();

    private final NetworkTable ASTable;
    private final double fieldXDimension = 16.540988;
    private final double fieldYDimension = 8.069326;
    private final double[] defaultPosition = {0.0, 0.0, 0.0};
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Kalman filter, experiment later
    private Matrix<N3, N1> visionStdDevs;
    private Matrix<N3, N1> stateStdDevs;

    private int totalTagCount = 0;

    // Inputs
    private Rotation2d gyroRotation;
    private SwerveModulePosition[] swerveModulePositions;

    // Outputs
    private StructPublisher<Pose2d> poseEstimatorEntry;

    /** 
     * Creates a new PoseEstimator. 
     */
    public PoseEstimator(Drivetrain drivetrain, Camera[] cameraArray)
    {
        super("PoseEstimator");
        System.out.println("  Constructor Started:  " + fullClassName);

        this.drivetrain = drivetrain;
        this.gyro = drivetrain.getPigeon2();
        this.cameraArray = cameraArray;

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        // This is where the robot starts in AdvantageScope
        poseEstimatorEntry = ASTable.getStructTopic("PoseEstimator", Pose2d.struct).publish();

        double[] doubleArray = {0.0, 0.0, 0.0};

        visionStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), doubleArray);
        stateStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), doubleArray);

        configStdDevs();

        if(drivetrain != null && gyro != null)
        {
            poseEstimator = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(), 
                gyro.getRotation2d(), 
                drivetrain.getState().ModulePositions,
                drivetrain.getState().Pose,
                stateStdDevs,
                visionStdDevs);

            drivetrain.setVisionMeasurementStdDevs(visionStdDevs);
            drivetrain.setStateStdDevs(stateStdDevs);
        }
        else
        {
            poseEstimator = null;
        }

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPose(pose);
    }

    /**
     * Tells the PoseEstimator how much to trust both the odometry and the vision.  Higher values indicate less trust for either the swerve states or the vision
     * check these values later when we know moore robot stuff
     */
    public void configStdDevs()
    {
        stateStdDevs.set(0, 0, 0.1); // x in meters
        stateStdDevs.set(1, 0, 0.1); // y in meters
        stateStdDevs.set(2, 0, 0.05); // heading in radians

        visionStdDevs.set(0, 0, 0.15); // x in meters // 0.2
        visionStdDevs.set(1, 0, 0.15); // y in meters // 0.2
        visionStdDevs.set(2, 0, 0.2); // heading in radians // 0.25
    }

    /**
     * gets the estimated pose updated by both the odometry and the vision
     * @return estimated pose
     */
    public Pose2d getEstimatedPose()
    {
        if(poseEstimator != null)
        {
            return estimatedPose;
        }
        else
        {
            return new Pose2d();
        }
    }

    public Supplier<Pose2d> getEstimatedPoseSupplier()
    {
        if(poseEstimator != null)
        {
            return () -> estimatedPose;
        }
        else
        {
            return () -> new Pose2d();
        }
    }

    /**
     * checks if the pose given is within the field boundaries in meters
     * @param pose
     * @return true or false
     */
    public boolean isPoseInsideField(Pose2d pose)
    {
        if((pose.getX() > -1.0 && pose.getX() < fieldXDimension + 1.0) && (pose.getY() > -1.0 && pose.getY() < fieldYDimension + 1.0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    Pose2d redHubPose = new Pose2d(new Translation2d(11.92, 4.030), new Rotation2d(0));
    Pose2d blueHubPose = new Pose2d(new Translation2d(4.62, 4.030), new Rotation2d(0));

    public DoubleSupplier getAngleToRedHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (redHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (redHubPose.getX() - robotPose.getX());
        DoubleSupplier rotation = () -> (Math.atan2((deltay.getAsDouble()), (deltax.getAsDouble())));
        return rotation;
    }

    // test this on 2025 bot and field
    public DoubleSupplier getAngleToBlueHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (blueHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (blueHubPose.getX() - robotPose.getX());
        DoubleSupplier rotation = () -> (Math.atan2((deltay.getAsDouble()), (deltax.getAsDouble())));
        return rotation;
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    /*
     * This method will be called once per scheduler run
     * Use this for sensors that need to be read periodically.
     * Use this for data that needs to be logged.
     */
    @Override
    public void periodic()
    {
        for(Camera camera : cameraArray)
        {
            if(gyro != null)
            {
                LimelightHelpers.SetRobotOrientation(camera.getCameraName(), drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            }

            // only update if we see 2+ tags
            if(camera.getTagCount() > 1)
            {
                Pose2d visionPose = camera.getPose();
                double robotVelo = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
                double robotRotation = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
                boolean rejectUpdate = false;

                if(visionPose == null)
                {
                    rejectUpdate = true;
                }

                if(robotVelo > 3.5)
                {
                    rejectUpdate = true;
                }

                if(robotRotation > 720.0)
                {
                    rejectUpdate = true;
                }

                if(!rejectUpdate)
                {
                    drivetrain.addVisionMeasurement(
                                visionPose,
                                camera.getTimestamp(),
                                visionStdDevs);
                }
            }
        }

        //OUTPUTS
        if(drivetrain != null && gyro != null && poseEstimator != null)
        {
            // grabs the newest estimated pose
            estimatedPose = drivetrain.getState().Pose;
            // sets it for advantagescope
            poseEstimatorEntry.set(estimatedPose);
        }
    }

    @Override
    public String toString()
    {
        return "Estimated Pose: " + getEstimatedPose();
    }
}