// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Hopper;
import frc.robot.sensors.HopperCamera;
import frc.robot.sensors.LaserCanSensor;
import frc.robot.sensors.RangerDistanceSensor;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
    // This string gets the full name of the class including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // Select the robot components to use
    private boolean useFullRobot                = false;

    private boolean useExampleSubsystem         = false;
    // private boolean useAgitator                 = false;
    private boolean useIndexigator              = false;
    private boolean useFlywheel                 = false;
    private boolean useIntake                   = false;
    private boolean useAccelerator              = false;
    private boolean useClimb                    = false;
    private boolean usePoseEstimator            = false;
    private boolean useDrivetrain               = false;

    private boolean useDriverController         = false;
    private boolean useOperatorController       = false;

    private boolean useLaserCAN                 = false;
    private boolean useRangerDistanceSensor     = false;
    private boolean useHopper                   = true;

    private boolean useHopperCamera             = false;
    private boolean useShooterCamera            = false;
    private boolean useIntakeCamera             = false;

    // Robot components
    private ExampleSubsystem exampleSubsystem = null;
    // private Agitator agitator = null;
    private Indexigator indexigator = null;
    private Flywheel flywheel = null;
    private Intake intake = null;
    private Accelerator accelerator;
    private Climb climb = null;
    private PoseEstimator poseEstimator = null;
    private Drivetrain drivetrain = null;
    private HopperCamera hopperCamera = null;
    private LEDs leds = null;
    private LaserCanSensor laserCanSensor = null;
    private RangerDistanceSensor rangerDistanceSensor = null;
    private Hopper hopper = null;

    private CommandXboxController driverController = null;
    private CommandXboxController operatorController = null;

    private final Camera[] cameraArray = new Camera[2];

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    RobotContainer() 
    {
        // Instantiate ONLY the components selected above
        if(useFullRobot || useExampleSubsystem)
            exampleSubsystem = new ExampleSubsystem();

        if(useFullRobot || useDrivetrain)
            drivetrain = TunerConstants.createDrivetrain();

        // if(useFullRobot || useAgitator)
        //     agitator = new Agitator(); 

        if(useFullRobot || useIndexigator)
            indexigator = new Indexigator();

        if(useFullRobot || useFlywheel)
            flywheel = new Flywheel();
        
        if(useFullRobot || useIntake)
            intake = new Intake();

        if(useFullRobot || useAccelerator)
            accelerator = new Accelerator();

        if(useFullRobot || useClimb)
            climb = new Climb();

        if(useFullRobot || useShooterCamera)
            cameraArray[0] = new Camera("limelight-shooter");
        else
            cameraArray[0] = null;
        
        if(useFullRobot || useIntakeCamera)
            cameraArray[1] = new Camera("limelight-intake");
        else
            cameraArray[1] = null;

        if(useFullRobot || useLaserCAN)
            laserCanSensor = new LaserCanSensor();
        
        if(useFullRobot || (useHopper))
            hopper = new Hopper();

        if(useFullRobot || useRangerDistanceSensor)
            rangerDistanceSensor = new RangerDistanceSensor();

        if(useFullRobot || useHopperCamera)
            hopperCamera = new HopperCamera(); 

        if(useFullRobot || usePoseEstimator)
            poseEstimator = new PoseEstimator(drivetrain, cameraArray);

        if(useFullRobot || useDriverController)
            driverController = new CommandXboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT);

        if(useFullRobot || useOperatorController)
            operatorController = new CommandXboxController(Constants.Controllers.OPERATOR_CONTROLLER_PORT);
            
        leds = new LEDs();
    }

    public ExampleSubsystem getExampleSubsystem()
    {
        return exampleSubsystem;
    }

    // public Agitator getAgitator()
    // {
    //     return agitator;
    // }

    public Indexigator getIndexigator()
    {
        return indexigator;
    }

    public Flywheel getFlywheel()
    {
        return flywheel;
    }

    public Intake getIntake()
    {
        return intake;
    }

    public Accelerator getAccelerator()
    {
        return accelerator;
    }

    public Climb getClimb()
    {
        return climb;
    }

    public PoseEstimator getPoseEstimator()
    {
        return poseEstimator;
    }

    public HopperCamera getHopperCamera()
    {
        return hopperCamera;
    }

    public Camera getShooterCamera()
    {
        return cameraArray[0];
    }

    public Camera getIntakeCamera()
    {
        return cameraArray[1];
    }

    public LEDs getLEDs()
    {
        return leds;
    }

    public Drivetrain getDrivetrain()
    {
        return drivetrain;
    }

    public CommandXboxController getDriverController()
    {
        return driverController;
    }

    public CommandXboxController getOperatorController()
    {
        return operatorController;
    }

    public LaserCanSensor getLaserCanSensor()
    {
        return laserCanSensor;
    }

    public Hopper getHopper()
    {
        return hopper;
    }

    public RangerDistanceSensor getRangerDistanceSensor()
    {
        return rangerDistanceSensor;
    }

    public boolean useFullRobot()
    {
        return useFullRobot;
    }
}
