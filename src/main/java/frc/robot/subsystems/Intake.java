package frc.robot.subsystems;

import static frc.robot.Constants.CANbus.*;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.TalonFXLance;

/**
 * This is the Intake Subsystem
 * @author Niyati
 */
public class Intake extends SubsystemBase
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


    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance intakePivotMotor = new TalonFXLance(1, ROBORIO, "Motor 1");
    private final TalonFXLance intakeRollersMotor = new TalonFXLance(2, ROBORIO, "Motor 2");
    private final TalonFXLance intakeRollersFollower = new TalonFXLance(3, ROBORIO, "Follow Motor");
    private double rollerPosition = 0.0;

    // private final double kP = 0.04;
    // private final double kI = 0.0;
    // private final double kD = 0.0;
    // private final double theshold = 4.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Intake Subsystem. 
     */
    public Intake()
    {
        super("Intake");
        System.out.println("  Constructor Started:  " + fullClassName);
        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        // Factory Default all motors
        intakePivotMotor.setupFactoryDefaults();
        intakeRollersMotor.setupFactoryDefaults();
        intakeRollersFollower.setupFactoryDefaults();

        // Sets up motors to be inverted or not (handle inversion on leaders)
        intakePivotMotor.setupInverted(true);
        intakeRollersMotor.setupInverted(true);

        // Sets up Brake Mode / Neutral Mode for leader + follower
        intakePivotMotor.setupBrakeMode();
        intakeRollersMotor.setupBrakeMode();
        intakeRollersFollower.setupBrakeMode();

        // Sets up position (encoder) for leader + follower
        intakePivotMotor.setPosition(0.0);
        intakeRollersMotor.setPosition(0.0);
        intakeRollersFollower.setPosition(0.0);

        // Set up Safety for leader + follower
        intakePivotMotor.setSafetyEnabled(false);
        intakeRollersMotor.setSafetyEnabled(false);
        intakeRollersFollower.setSafetyEnabled(false);

        // Configure the follower last so configurables above are not overwritten
        // Make device 3 follow device 2 with inverted alignment (opposed sides)
        intakeRollersFollower.setupFollower(2, true);
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    public void set(double speed)
    {
        intakePivotMotor.set(speed);
        intakeRollersMotor.set(speed);
    }

    public void setVoltage(double voltage)
    {
        intakePivotMotor.setVoltage(voltage);
        intakeRollersMotor.setVoltage(voltage);
    }

    public void pickUpFuel()
    {
        intakePivotMotor.setupCoastMode();
        intakeRollersMotor.setupCoastMode();
        intakeRollersFollower.setupCoastMode();
        set(-0.2);
    }

    public void ejectFuel()
    {
       intakePivotMotor.setupCoastMode();
       intakeRollersMotor.setupCoastMode();
       intakeRollersFollower.setupCoastMode();
       set(0.2);
    }
    public void stop()
    {
        intakePivotMotor.set(0.0);
        intakeRollersMotor.set(0.0);
    }

    public Command pickupFuelCommand()
    {
        return run(() -> pickUpFuel()).withName("Pickup Fuel");
    }

    public Command ejectCoralCommand() 
    {
        return run(() -> ejectFuel()).withName("Eject Fuel");
    }

    public Command onCommand()
    {
        return run( () -> set(0.25) );
    }

    public Command setCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return runOnce(() -> stop()).withName("Stop Intake");
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
    }

    @Override
    public String toString()
    {
        return "Current Intake Position: " + rollerPosition;
    }
}
