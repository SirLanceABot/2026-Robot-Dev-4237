package frc.robot.commands;

import java.lang.invoke.MethodHandles;

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

    private static Drivetrain drivetrain;
    private static LEDs leds;
    private static Notifier notifier; // background timer

    private static boolean currentlyBlinking = false;
    private static boolean monitorEnabled = false;

    // private static Command selectedCommand = null;

    // tolerance for wheel angle to be considered "forward" (degrees)
    private static final double TOLERANCE_DEGREES = 5.0;
    // tolerance for gyro
    private static final double GYRO_TOLERANCE_DEGREES = 5.0;
    // check period in seconds
    private static final double PERIOD_S = 0.5;

    private StartUpCommands() 
    {}

    /** 
     * This method enables the monitoring (starts notifier)
     */
    public static void enableMonitor(RobotContainer robotContainer)
    {
        drivetrain = robotContainer.getDrivetrain();
        leds = robotContainer.getLEDs();
        // if (drivetrain == null || leds == null)
        // {
        //     System.out.println("StartUpCommands: drivetrain or leds null, not starting monitor");
        //     return;
        // }

        if (drivetrain != null && drivetrain.getPigeon2() != null)
        {
            drivetrain.getPigeon2().setYaw(0.0);
            System.out.println("StartUpCommands - Pigeon zeroed");
        }

        if (notifier == null)
        {
            notifier = new Notifier(StartUpCommands::checkAndUpdate);
            notifier.startPeriodic(PERIOD_S);
            System.out.println("StartUpCommands - notifier started");
        }

        monitorEnabled = true;
        System.out.println("StartUpCommands - monitor enabled");
    }

    /**
     * This method will disable the monitor
     */
    public static void disableMonitor()
    {
        monitorEnabled = false;

        if (notifier != null)
        {
            notifier.stop();
            notifier = null;
            System.out.println("StartUpCommands - notifier stopped");
        }
    }

    public static void checkAndUpdate()
    {    
        if (!monitorEnabled)
            return;

        if (isLowVoltage())
            return;

        if (isGyroMoved())
            return;

        isSwerveAligned();
    }

    /** 
     * Check bettery volatge and set LEDs to yellow if below theshold, and return true is battery handled 
     */
    private static boolean isLowVoltage()
    {
        double voltage = RobotController.getBatteryVoltage();
        if (leds != null && voltage < Constants.Power.BATTERY_THRESHOLD_VOLTS)
        {
            System.out.println("StartUpCommands: battery low (" + voltage + " V) - setting LEDs yellow");
            Command yellow = leds.setColorBlinkCommand(Color.kYellow);
            
            if (yellow != null)
                yellow.ignoringDisable(true).schedule();

            return true;
        }
        
        return false;
    }

    /**
     * Check swerve module angles and set leds to red is not aligned and green is aligned
     */
    private static boolean isSwerveAligned()
    {
        if (drivetrain == null)
        {
            return false;
        }

        // each swerve module contains wheel distance travelled and wheel angle (rotation2d)
        SwerveModulePosition[] modules = drivetrain.getModuleStates();
        if (modules == null || modules.length == 0)
        {
            return false;
        }

        boolean anyMisaligned = false;
        double tolRad = Math.toRadians(TOLERANCE_DEGREES); // converting tolerance to radians for WPILab angles

        // Report all misaligned modules with index and angles for easier debugging
        for (int i = 0; i < modules.length; ++i)
        {
            SwerveModulePosition m = modules[i];
            Rotation2d angle = m.angle;
            double angleD = angle.getRadians();

            // normalize to [0, pi) positive angle
            // atan2(sin, cos) = normalizes angle into [-π, π]
            // abs() = now [0, π]
            // Math.min(absNorm, π - absNorm) = distance to nearest forward direction
            double absNorm = Math.abs(Math.atan2(Math.sin(angleD), Math.cos(angleD)));
            double angleToNearestPi = Math.min(absNorm, Math.PI - absNorm);

            // checking against tolerance
            if (angleToNearestPi > tolRad)
            {
                anyMisaligned = true;
                // used to find angle of misalignment but not needed
                // double moduleAngleDeg = Math.toDegrees(angleD);
                // double degFromForward = Math.toDegrees(angleToNearestPi);
                // System.out.println(("StartUpCommands: module " + i));
            }
        }

        if (anyMisaligned && !currentlyBlinking)
        {
            if (!currentlyBlinking)
            {
                currentlyBlinking = true;
                System.out.println("StartUpCommands: wheels misaligned = making LEDs blink");
                Command red = leds.setColorBlinkCommand(Color.kRed); // red = swerve not lined up

                // if not already blinking
                if (red != null) 
                {
                    red.ignoringDisable(true).schedule();
                }
            }
        }
        else if (!anyMisaligned && currentlyBlinking)
        {
            currentlyBlinking = false;
            Command green = leds.setColorSolidCommand(100, Color.kGreen);
            if (green != null)
            {
                green.ignoringDisable(true).schedule();
            }
        }

        return !anyMisaligned;
        // else // all wheels aligned
        // {
        //     // if blinking
        //     if (currentlyBlinking)
        //     {
        //         currentlyBlinking = false;
        //         System.out.println("StartUpCommands: wheels aligned = making LEDs green");

        //         Command solid = leds.setColorSolidCommand(100, Color.kGreen);
        //         if (solid != null) 
        //             solid.ignoringDisable(true).schedule();

        //         // keep notifier running so we continue monitoring while disabled
        //         // if (notifier != null) 
        //         //     notifier.stop();
        //     }
        // }

        // leds.offCommand();
    }

    /** 
     * This method will check gyro rotation from zero and turns leds orange if past tolerance
     */
    private static boolean isGyroMoved()
    {
        if (drivetrain == null || drivetrain.getPigeon2() == null || leds == null)
        {
            return false;
        }

        double yawDegrees = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double absYaw = Math.abs(yawDegrees);

        if (absYaw > GYRO_TOLERANCE_DEGREES)
        {
            System.out.println("StartUpCommands: gyro moved (" + yawDegrees + " deg) - Setting LEDs orange");

            Command orange = leds.setColorBlinkCommand(Color.kOrange);
            if (orange != null)
            {
                orange.ignoringDisable(true).schedule();
            }

            return true;
        }

        return false;
    }
}
