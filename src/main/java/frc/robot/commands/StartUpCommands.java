// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    private static RobotContainer robotContainer; // should this be final?
    private static Drivetrain drivetrain;
    private static LEDs leds;
    private static Notifier notifier; // background timer
    private static volatile boolean currentlyBlinking = false;
    private static Command selectedCommand = null;

    // tolerance for wheel angle to be considered "forward" (degrees)
    private static final double TOLERANCE_DEGREES = 5.0;
    // check period in seconds
    private static final double PERIOD_S = 0.5;

    private StartUpCommands() 
    {}

    /** This function starts the startup monitor that checks wheel alignment and updates LEDs.
     * @param robotContainer
     */
    public static void startMonitor(RobotContainer robotContainer)
    {
        drivetrain = robotContainer.getDrivetrain();
        leds = robotContainer.getLEDs();

        if (drivetrain == null || leds == null)
        {
            System.out.println("StartUpCommands: drivetrain or leds null, not starting monitor");
            return;
        }

        // run an immediate check and then a periodic notifier until aligned
        checkAndUpdate();

        notifier = new Notifier(StartUpCommands::checkAndUpdate);
        notifier.startPeriodic(PERIOD_S);
    }

    private static void checkAndUpdate()
    {        
        // Check battery voltage first. If battery is low, force LEDs to yellow and skip other
        // startup logic so the low-battery state is clearly visible.
        double voltage = RobotController.getBatteryVoltage();
        if (leds != null && voltage < Constants.Power.BATTERY_THRESHOLD_VOLTS)
        {
            System.out.println("StartUpCommands: battery low (" + voltage + " V) - setting LEDs yellow");
            Command yellow = leds.setColorSolidCommand(100, Color.kYellow);
            
            if (yellow != null)
                yellow.ignoringDisable(true).schedule();

            return; // don't override battery warning with other states
        }

        // Only should run checks while the robot is disabled so we continuously watch during disabled mode
        if (!DriverStation.isDisabled())
        {
            // if(leds != null)
            // {
            //     if(!robotContainer.useFullRobot())
            //     {
            //         leds.setColorSolidCommand(100, Color.kYellow);
            //     }
            //     else if(selectedCommand != null)
            //     {
            //         leds.setColorSolidCommand(100, Color.kGreen);
            //     }
            //     else
            //     {
            //         leds.setColorSolidCommand(100, Color.kYellow);
            //     }
            // }

            // if(leds != null)
            // {
            //     leds.setColorSolidCommand(100, Color.kRed);
            // }

            return;
        }

        // each swerve module contains wheel distance travelled and wheel angle (rotation2d)
        SwerveModulePosition[] modules = drivetrain.getModuleStates();
        if (modules == null || modules.length == 0)
        {
            return;
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

        if (anyMisaligned)
        {
            if (!currentlyBlinking)
            {
                currentlyBlinking = true;
                System.out.println("StartUpCommands: wheels misaligned = making LEDs blink");
                Command blink = leds.setColorSolidCommand(100, Color.kRed); // red = swerve not lined up
                
                // if not already blinking
                if (blink != null) 
                    blink.ignoringDisable(true).schedule();
            }
        }
        else // all wheels aligned
        {
            // if blinking
            if (currentlyBlinking)
            {
                currentlyBlinking = false;
                System.out.println("StartUpCommands: wheels aligned = making LEDs green");
                Command solid = leds.setColorSolidCommand(100, Color.kGreen);
                if (solid != null) 
                    solid.ignoringDisable(true).schedule();

                // keep notifier running so we continue monitoring while disabled
                // if (notifier != null) 
                //     notifier.stop();
            }
        }

        // leds.offCommand();
    }
}
