package frc.robot.tests;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

@SuppressWarnings("unused")
public class MatthewFTest implements Test
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }


    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here



    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    private final RobotContainer robotContainer;
    private final Drivetrain drivetrain;
    private final Joystick joystick = new Joystick(0);
    private DoubleSupplier start = () -> 0.0;
    private DoubleSupplier end = () -> Math.PI / 4.0;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public MatthewFTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        drivetrain = robotContainer.getDrivetrain();

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    
    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
      

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
        //TODO for angleLock drive based on gyro
        // drivetrain.getPigeon2().reset();
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        //angleLock drive in test
        if(joystick.getRawButton(1))
        {
            drivetrain.angleLockDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0), () -> 0.1, end).schedule();

        }
        else
        {
            drivetrain.angleLockDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0), () -> 0.1, start).schedule();
        }
        System.out.println("Angle: " + drivetrain.getPigeon2().getRotation2d().getRadians());

        //TODO for angleLock drive based on gyro
        // if(joystick.getRawButton(1))
        // {
        //     drivetrain.angleLockDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0), () -> 0.1, drivetrain.getPigeon2().getRotation2d().plus(end)).schedule();
        // }
        // else
        // {
        //     drivetrain.angleLockDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0), () -> 0.1, drivetrain.getPigeon2().getRotation2d().plus(start)).schedule();
        // }

    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        Commands.none().schedule();
    } 
}
