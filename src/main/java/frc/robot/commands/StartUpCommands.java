package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import com.fasterxml.jackson.databind.EnumNamingStrategies.LowerCamelCaseStrategy;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants;
import frc.robot.sensors.CANRange;

/**
 * This class checks the swerves at startup and blinks leds red until tehy are aligned.
 */
public final class StartUpCommands
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public enum StartUpState
    {
        LOW_VOLTAGE, GYRO_NOT_ZEROED, CANRANGE_OFF, SWERVE_MISALIGNED, READY
    }

    private static Drivetrain drivetrain;
    private static LEDs leds;
    private static CANRange CANRange0;
    private static CANRange CANRange1;

    private static Notifier notifier; // background timer

    // private static boolean currentlyBlinking = false;
    // private static boolean monitorEnabled = false;
    public static StartUpState currentState = null;
    private static boolean running = false;

    // private static Command selectedCommand = null;

    // tolerance for wheel angle to be considered "forward" (degrees)
    private static final double SWERVE_TOLERANCE_DEGREES = 3.0;
    // tolerance for gyro
    private static final double GYRO_TOLERANCE_DEGREES = 3.0;
    // check period in seconds
    private static final double PERIOD_S = 0.5;

    private StartUpCommands() 
    {}

    public static StartUpState getCurrentState()
    {
        return currentState;
    }

    /**
     * This method will determind what state the robot shoudl be in 
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
     * This method will update leds for states 
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

            case CANRANGE_OFF:
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

    // /** 
    //  * This method enables the monitoring (starts notifier)
    //  */
    public static void enableMonitor(RobotContainer robotContainer)
    {
        if (running)
        {
            return;
        }

        running = true;

        drivetrain = robotContainer.getDrivetrain();
        leds = robotContainer.getLEDs();
        CANRange0 = robotContainer.getCANrange(0);
        CANRange1 = robotContainer.getCANrange(1);



        if (drivetrain != null && drivetrain.getPigeon2() != null)
        {
            System.out.println("StartUpCommands - Pigeon zeroed");
        }

        notifier = new Notifier(StartUpCommands::checkAndUpdate);
        notifier.startPeriodic(PERIOD_S);

        System.out.println("StartUpCommands - StartUp checks running");
    }

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

    public static void checkAndUpdate()
    {    
        System.out.println("StartUpCommands go");
        StartUpState State = runStartUpChecks();

        if (State != currentState)
        {
            System.out.println("State changed to " + State);
            currentState = State;
            updateLEDsForState(State);
        }
    }

    /** 
     * Check bettery volatge and set LEDs to yellow if below theshold, and return true is battery handled 
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
     * Check swerve module angles and set leds to red is not aligned and green is aligned
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
                System.out.println("StartUpCommands - Swerve misaligned (module " + i + ")");
                return StartUpState.SWERVE_MISALIGNED;
            }
        }

        return null;
    }


    /** 
     * This method will check gyro rotation from zero and turns leds orange if past tolerance
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
            System.out.println("StartUpCommands: gyro moved (" + yawDegrees + " deg) - Setting LEDs orange");

            return StartUpState.GYRO_NOT_ZEROED;
        }

        return null;
    }

    // /**
    //  * This method will check if the CANRanges is returning anything
    //  */
    private static StartUpState checkCANRanges()
    {
        if (CANRange0 != null || CANRange1 != null)
        {
            if (CANRange0 == null) // What else does this return dawg
            {
                System.out.println("StartUpCommands - CANRange0 is not working");
                return StartUpState.CANRANGE_OFF;
            }

            if (CANRange1 == null)
            {
                System.out.println("StartUpCommands - CANRange1 is not working");
                return StartUpState.CANRANGE_OFF;
            }
        }

        return null;
    }
}
