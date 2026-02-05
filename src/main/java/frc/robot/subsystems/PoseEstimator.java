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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
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

    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final int ITERATIONS = 3; // number of iterations for time of flight to actual target and calculated target to converge

    //For purelyCalculatedLeadingAngle calculations
    private final double shooterAngleRadians = (72.0/360.0)*(2.0*Math.PI);  
    private final double robotToHubVerticalDistanceMeters = -1.2954;

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
        configTimeOfFlightMap();


        if(drivetrain != null && gyro != null && cameraArray != null)
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

    private void configTimeOfFlightMap()
    {
        // time of flight map, where the first value is distance to hub in feet, second is time fuel is in the air
        // TODO test time values once we have robot
        timeOfFlightMap.put(3.0, 0.70);
        timeOfFlightMap.put(4.0, 0.75);
        timeOfFlightMap.put(5.0, 0.775);
        timeOfFlightMap.put(6.0, 0.80);
        timeOfFlightMap.put(7.0, 0.825);
        timeOfFlightMap.put(8.0, 0.85);
        timeOfFlightMap.put(9.0, 0.875);
        timeOfFlightMap.put(10.0, 0.90);
        timeOfFlightMap.put(11.0, 0.925);
        timeOfFlightMap.put(12.0, 0.95);
        timeOfFlightMap.put(13.0, 0.975);
        timeOfFlightMap.put(14.0, 1.0);
        timeOfFlightMap.put(15.0, 1.025);
        timeOfFlightMap.put(16.0, 1.05);
        timeOfFlightMap.put(17.0, 1.075);
        timeOfFlightMap.put(18.0, 1.10);
        timeOfFlightMap.put(19.0, 1.125);
        timeOfFlightMap.put(20.0, 1.15);
    }

    public double getTOF(double dist)
    {
        dist = Math.max(4.0, Math.min(20.0, dist));
        return timeOfFlightMap.get(dist);
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
        stateStdDevs.set(0, 0, 0.2); // x in meters
        stateStdDevs.set(1, 0, 0.2); // y in meters
        stateStdDevs.set(2, 0, 0.25); // heading in radians

        visionStdDevs.set(0, 0, 0.2); // x in meters // 0.2
        visionStdDevs.set(1, 0, 0.2); // y in meters // 0.2
        visionStdDevs.set(2, 0, 0.25); // heading in radians // 0.25
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

    public Pose2d getRedHubPose()
    {
        return redHubPose;
    }

    public Pose2d getBlueHubPose()
    {
        return blueHubPose;
    }

    /**
     * gets the pose of the hub of your alliance (ideally)
     * @return hub pose
     */
    public Pose2d getAllianceHubPose()
    {
        if(drivetrain.isRedAllianceSupplier().getAsBoolean())
        {
            return getRedHubPose();
        }
        else
        {
            return getBlueHubPose();
        }
    }

    public DoubleSupplier getDistanceToRedHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (redHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (redHubPose.getX() - robotPose.getX());
        DoubleSupplier dist = () -> Math.hypot(deltax.getAsDouble(), deltay.getAsDouble());
        return dist;
    }

    public DoubleSupplier getDistanceToBlueHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (blueHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (blueHubPose.getX() - robotPose.getX());
        DoubleSupplier dist = () -> Math.hypot(deltax.getAsDouble(), deltay.getAsDouble());
        return dist;
    }

    /**
     * gets the distance from robot pose to the target provided
     * @param robotPose self explanatory
     * @param target target pose
     * @return distance
     */
    public DoubleSupplier getDistanceToTarget(Pose2d robotPose, Pose2d target)
    {
        DoubleSupplier deltay = () -> (target.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (target.getX() - robotPose.getX());
        return () -> Math.hypot(deltax.getAsDouble(), deltay.getAsDouble());
    }

    /**
     * gets the distance to the hub of your alliance (ideally)
     * @return distance to hub, in meters
     */
    public DoubleSupplier getDistanceToAllianceHub()
    {
        if(drivetrain.isRedAllianceSupplier().getAsBoolean())
        {
            return getDistanceToRedHub();
        }
        else
        {
            return getDistanceToBlueHub();
        }
    }

    public DoubleSupplier getAngleToRedHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (redHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (redHubPose.getX() - robotPose.getX());
        DoubleSupplier rotation = () -> (Math.atan2((deltay.getAsDouble()), (deltax.getAsDouble())));
        return rotation;
    }

    // tested on 2025 bot and works now
    public DoubleSupplier getAngleToBlueHub()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        DoubleSupplier deltay = () -> (blueHubPose.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (blueHubPose.getX() - robotPose.getX());
        DoubleSupplier rotation = () -> (Math.atan2((-deltay.getAsDouble()), (-deltax.getAsDouble())));
        return rotation;
    }

    /**
     * gets the rotation needed to make the robot face the alliance hub directly
     * TESTED ON 2025 BOT AND WORKS
     * @return the angle to rotate to, in radians
     */
    public DoubleSupplier getAngleToAllianceHub()
    {
        if(drivetrain.isRedAllianceSupplier().getAsBoolean())
        {
            return getAngleToRedHub();
        }
        else
        {
            return getAngleToBlueHub();
        }
    }

    public DoubleSupplier getAngleToRedTarget(Pose2d robotPose, Pose2d target)
    {
        DoubleSupplier deltay = () -> (target.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (target.getX() - robotPose.getX());
        return () -> Math.atan2(deltay.getAsDouble(), deltax.getAsDouble());
    }

    public DoubleSupplier getAngleToBlueTarget(Pose2d robotPose, Pose2d target)
    {
        DoubleSupplier deltay = () -> (target.getY() - robotPose.getY());
        DoubleSupplier deltax = () -> (target.getX() - robotPose.getX());
        return () -> Math.atan2(-deltay.getAsDouble(), -deltax.getAsDouble());
    }

    /**
     * gets the pose of the calculated target for shoot on the move while the robot is in motion. 
     * if the robot is not moving, the calculated target should be the same as the actual target (alliance hub)
     * @return the calculated target translation to shoot at
     * @author biggie cheese
     */
    public Pose2d getCalculatedTargetPose(Pose2d actualTarget, Pose2d robotPose, ChassisSpeeds velocity)
    {
        Translation2d targetTranslation = actualTarget.getTranslation();
        Translation2d robotTranslation = robotPose.getTranslation();

        Translation2d calculatedTargetTranslation = targetTranslation;

        for(int i = 0; i < ITERATIONS; i++)
        {
            double distance = robotTranslation.getDistance(calculatedTargetTranslation);

            double tof = getTOF(distance);

            double xoffset = -tof * velocity.vxMetersPerSecond;
            double yoffset = -tof * velocity.vyMetersPerSecond;

            calculatedTargetTranslation = new Translation2d(
                targetTranslation.getX() + xoffset,
                targetTranslation.getY() + yoffset);
        }

        return new Pose2d(calculatedTargetTranslation, new Rotation2d());
    }

    /**
     * gets the rotation to the calculated target for shoot on the move
     * @return the target heading, in radians
     * @author biggie cheese
     */
    public DoubleSupplier getRotationToCalculatedTarget()
    {
        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getRobotRelativeSpeeds(), robotPose.getRotation());

        Pose2d calculatedTarget = getCalculatedTargetPose(
            getAllianceHubPose(), 
            robotPose, 
            velocity);

        if(drivetrain.isRedAllianceSupplier().getAsBoolean())
        {
            DoubleSupplier targetHeading = () -> (getAngleToRedTarget(robotPose, calculatedTarget).getAsDouble());
            return targetHeading;
        }
        else
        {
            DoubleSupplier targetHeading = () -> (getAngleToBlueTarget(robotPose, calculatedTarget).getAsDouble());
            return targetHeading;
        }
    }


    /**
     * Mathematically "pure" leading angle calculation
     * Calculates leading angle based off time of flight and shooter velocity (no InterpolatingDoubleTreeMap)
     * @param target : the hub you want to shoot at
     * @return target angle, in radians
     * @author Matthew
     */
    public DoubleSupplier pureLeadingAngle(Pose2d target)
    {
        return () ->
        {
            Pose2d robotPose = drivetrain.getState().Pose;            

            double xVelocityField = (drivetrain.getState().Speeds.vxMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getCos()) - drivetrain.getState().Speeds.vyMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getSin()));
            double yVelocityField = (drivetrain.getState().Speeds.vxMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getSin()) + drivetrain.getState().Speeds.vyMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getCos()));

            double deltax = target.getX() - robotPose.getX();
            double deltay = target.getY() - robotPose.getY();
            
            double distanceFromTarget = 0.0;
            double velocity = 0.0;
            double timeOfFlight = 0.0;
            double xDelta = deltax;
            double yDelta = deltay;

            for(int i = 0; i < 3; i++)
            {
                distanceFromTarget = Math.hypot(xDelta, yDelta);
                velocity = (distanceFromTarget / Math.cos(shooterAngleRadians)) * Math.sqrt((9.8) / (2 * (robotToHubVerticalDistanceMeters + distanceFromTarget * Math.tan(shooterAngleRadians))));
                timeOfFlight = ((velocity * Math.sin(shooterAngleRadians)) + Math.sqrt(Math.pow(velocity * Math.sin(shooterAngleRadians), 2) + 2 * 9.8 * robotToHubVerticalDistanceMeters)) / 9.8;

                xDelta = deltax - (xVelocityField * timeOfFlight);
                yDelta = deltay - (yVelocityField * timeOfFlight);
                // System.out.println("yDelta: " + yDelta + " xDelta: " + xDelta);
            }
            return Math.atan2(yDelta, xDelta);
        };
    }

    /**
     * Mathematically "pure" shooter velocity calculation (for leading angle)
     * Calculates shooter velocity (no InterpolatingDoubleTreeMap)
     * @param target : the hub you want to shoot at
     * @return target shooter velocity for the leading angle
     * @author Matthew
     */
    public DoubleSupplier pureShooterVelocity(Pose2d target)
    {
        return () ->
        {
            Pose2d robotPose = drivetrain.getState().Pose;            

            double xVelocityField = (drivetrain.getState().Speeds.vxMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getCos()) - drivetrain.getState().Speeds.vyMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getSin()));
            double yVelocityField = (drivetrain.getState().Speeds.vxMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getSin()) + drivetrain.getState().Speeds.vyMetersPerSecond * Math.abs(drivetrain.getState().Pose.getRotation().getCos()));

            double deltax = target.getX() - robotPose.getX();
            double deltay = target.getY() - robotPose.getY();
            
            double distanceFromTarget = 0.0;
            double velocity = 0.0;
            double timeOfFlight = 0.0;
            double xDelta = deltax;
            double yDelta = deltay;

            for(int i = 0; i < 3; i++)
            {
                distanceFromTarget = Math.hypot(xDelta, yDelta);
                velocity = (distanceFromTarget / Math.cos(shooterAngleRadians)) * Math.sqrt((9.8) / (2 * (robotToHubVerticalDistanceMeters + distanceFromTarget * Math.tan(shooterAngleRadians))));
                timeOfFlight = ((velocity * Math.sin(shooterAngleRadians)) + Math.sqrt(Math.pow(velocity * Math.sin(shooterAngleRadians), 2) + 2 * 9.8 * robotToHubVerticalDistanceMeters)) / 9.8;

                xDelta = deltax - (xVelocityField * timeOfFlight);
                yDelta = deltay - (yVelocityField * timeOfFlight);
            }
            return velocity;
        };
    }

    

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    boolean isFirstTagSight = true;
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
            if(camera != null)
            {
                if(gyro != null && drivetrain != null)
                {
                    LimelightHelpers.SetRobotOrientation(camera.getCameraName(), drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
                }

                // only update if we see 2+ tags
                if(camera.getTagCount() > 0)
                {
                    Pose2d visionPose = camera.getPose();
                    double robotVelo = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
                    double robotRotation = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
                    boolean rejectUpdate = false;

                    if(isFirstTagSight && visionPose != null)
                    {
                        isFirstTagSight = false;
                        drivetrain.resetPose(visionPose);
                    }

                    if(visionPose == null)
                    {
                        rejectUpdate = true;
                    }

                    if(robotVelo > 3.5)
                    {
                        rejectUpdate = true;
                    }

                    if(robotRotation > 360.0)
                    {
                        rejectUpdate = true;
                    }

                    if(!rejectUpdate)
                    {
                        drivetrain.addVisionMeasurement(
                                    visionPose,
                                    camera.getTimestamp(),
                                    visionStdDevs);
                        drivetrain.resetPose(new Pose2d(visionPose.getTranslation(), drivetrain.getState().RawHeading));
                    }
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