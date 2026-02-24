package frc.robot.commands;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.sensors.Hopper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants;


/**
 * This class essentially runs a robot health check while the robot is disabled before a match.
 * 
 * It runs repeatedly in the background and:
 *      checks hardware stuff
 *      shows problems using led colors
 *      stops checking when robot leaves disabled
 * 
 * Robot checks in this order:
 *      Battery voltage
 *      Gyro zeroed
 *      CANRange sensors detected once
 *      Swerve modules aligned forward
 * 
 * Only first failing check will be shown on leds 
 */
public final class StartUpCommands
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    /**
     * Robot startup status
     * used by leds and elastic alerts
     */
    public enum StartUpState
    {
        LOW_VOLTAGE, 
        GYRO_NOT_ZEROED, 
        CAN_RANGE_OFF, 
        SWERVE_MISALIGNED, 
        READY
    }

    private static Drivetrain drivetrain;
    private static LEDs leds;

    private static Hopper hopper;

    /** Background loop that runs check every PERIOD_S seconds */
    private static Notifier notifier;

    /** Current system health state */
    public static StartUpState currentState = null;

    private static boolean running = false;

    /** 
     * Once sensors detect something one time.
     * we set them verified and stop requiring detection
     * this is what lets the "hand in front of sensor" test.
     */
    private static boolean canRangeVerified = false;

    /* Boolean suppliers from Hopper subsystem */
    private static BooleanSupplier LeftCam;
    private static BooleanSupplier RightCam;

    /** Wheel must face forward within this tolerance */
    private static final double SWERVE_TOLERANCE_DEGREES = 3.0;

    /** Gyro must be near zero within this tolerance */
    private static final double GYRO_TOLERANCE_DEGREES = 3.0;

    /** How often checks run (seconds) */
    private static final double PERIOD_S = 0.5;

    private StartUpCommands() 
    {}

    public static StartUpState getCurrentState()
    {
        return currentState;
    }

    /**
     * Runs checks in priority order
     * Returns first failure found
     * Returns ready is everything passes
     * @return
     */
    private static StartUpState runStartUpChecks()
    {
        StartUpState result = null;

        // First - Voltage check
        result = checkBattery();
        if (result != null)
        {
            return result;
        }

        // Second - Gyro Check
        result = checkGyro();
        if (result != null)
        {
            return result;
        }

        // Third - CAN Range check
        result = checkCANRanges();
        if (result != null)
        {
            return result;
        }

        // Fourth - Swerve alignment
        result = checkSwerve();
        if (result != null)
        {
            return result;
        }

        return StartUpState.READY;
    }

    /**
     * Sets LED pattern based on robot state
     */
    private static void updateLEDsForState(StartUpState State)
    {
        if (leds == null)
        {
            return;
        }

        Command command = null;

        switch (State)
        {
            case LOW_VOLTAGE:
                command = leds.setColorBlinkCommand(Color.kYellow);
                break;

            case GYRO_NOT_ZEROED:
                command = leds.setColorBlinkCommand(Color.kOrange);
                break;

            case CAN_RANGE_OFF:
                command = leds.setColorSolidCommand(80, Color.kPurple);
                break;

            case SWERVE_MISALIGNED:
                command = leds.setColorBlinkCommand(Color.kRed);
                break;

            case READY:
                command = leds.setMovingRainbowCommand();
                break;
        }

        if (command != null)
        {
            command.ignoringDisable(true).schedule();
        }
    }

    /**
     * Starts background monitoring
     * Called when robot enters disabled during prematch
     */
    public static void enableMonitor(RobotContainer robotContainer)
    {
        if (running)
        {
            return;
        }

        running = true;

        drivetrain = robotContainer.getDrivetrain();
        leds = robotContainer.getLEDs();
        hopper = robotContainer.getHopper();
        
        if(hopper != null)
        {
            LeftCam = hopper.isLeftFullSupplier();
            RightCam = hopper.isRightFullSupplier();
        }
        

        if (drivetrain != null && drivetrain.getPigeon2() != null)
        {
            System.out.println("StartUpCommands - Pigeon zeroed");
        }

        notifier = new Notifier(StartUpCommands::checkAndUpdate);
        notifier.startPeriodic(PERIOD_S);

        System.out.println("StartUpCommands - StartUp checks running");
    }

    /**
     * Stops backgroung monitoring
     */
    public static void disableMonitor()
    {
        if (!running)
        {
            return;
        }

        running = false;

        if (notifier != null)
        {
            notifier.stop();
            notifier = null;
            System.out.println("StartUpCommands - StartUp checks stopped");
        }
    }

    /**
     * Runs checks and updates LEDs if state changes
     */
    public static void checkAndUpdate()
    {    
        // System.out.println("StartUpCommands go");
        StartUpState State = runStartUpChecks();

        if (State != currentState)
        {
            System.out.println("State changed to " + State);
            currentState = State;
            updateLEDsForState(State);
        }
    }

    /** 
     * Battery must be above configured threshold
     */
    private static StartUpState checkBattery()
    {
        double voltage = RobotController.getBatteryVoltage();

        if (leds != null && voltage < Constants.Power.BATTERY_THRESHOLD_VOLTS)
        {
            System.out.println("StartUpCommands: battery low (" + voltage + " V) - setting LEDs yellow");

            return StartUpState.LOW_VOLTAGE;
        }
        
        return null;
    }

    /** 
     * Gyro must be near zero before match starts
     */
    private static StartUpState checkGyro()
    {
        if (drivetrain == null || drivetrain.getPigeon2() == null)
        {
            return null;
        }

        double yawDegrees = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double absYaw = Math.abs(yawDegrees);

        if (absYaw > GYRO_TOLERANCE_DEGREES)
        {
            // System.out.println("StartUpCommands: gyro moved (" + yawDegrees + " deg) - Setting LEDs orange");

            return StartUpState.GYRO_NOT_ZEROED;
        }

        return null;
    }

    /**
     * CANRange sensor verification logic
     * 
     * Robot only needs to detect something one time
     * After detection, sensors are considered working
     * 
     * Allows manual hand test during startup
     */
    private static StartUpState checkCANRanges()
    {
        if (drivetrain == null)
        {
            return null;
        }

        if (LeftCam == null || RightCam == null)
        {
            return StartUpState.CAN_RANGE_OFF;
        }

        boolean leftDetected = LeftCam.getAsBoolean();
        boolean rightDetected = RightCam.getAsBoolean();

        if (!leftDetected || rightDetected)
        {
            System.out.println("StartUpCommands - CANRanged verified");
            canRangeVerified = true; 
            return null;
        }

        return StartUpState.CAN_RANGE_OFF;
    }

    /**
     * All swerve wheels must face forward before match
     */
    private static StartUpState checkSwerve()
    {
        if (drivetrain == null)
        {
            return null;
        }
        // each swerve module contains wheel distance travelled and wheel angle (rotation2d)
        SwerveModulePosition[] modules = drivetrain.getModuleStates();
        if (modules == null || modules.length == 0)
        {
            return null;
        }

        double tolRad = Math.toRadians(SWERVE_TOLERANCE_DEGREES); // converting tolerance to radians for WPILab angles

        // Report all misaligned modules with index and angles for easier debugging
        for (int i = 0; i < modules.length; i++)
        {

            Rotation2d angle = modules[i].angle;
            double angleRad = angle.getRadians();

            // normalize to [0, pi) positive angle
            // atan2(sin, cos) = normalizes angle into [-π, π]
            // abs() = now [0, π]
            // Math.min(absNorm, π - absNorm) = distance to nearest forward direction
            double absNorm = Math.abs(Math.atan2(Math.sin(angleRad), Math.cos(angleRad)));
            double angleToNearestPi = Math.min(absNorm, Math.PI - absNorm);

            // checking against tolerance
            if (angleToNearestPi > tolRad)
            {
                // System.out.println("StartUpCommands - Swerve misaligned (module " + i + ")");
                return StartUpState.SWERVE_MISALIGNED;
            }
        }

        return null;
    }
}
