package frc.robot.subsystems;

import static frc.robot.Constants.Intake.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    private final TalonFXLance intakePivotMotor = new TalonFXLance(INTAKEPIVOTMOTOR, MOTOR_CAN_BUS, "Pivot Motor");
    private final TalonFXLance intakeRollersMotor = new TalonFXLance(INTAKEROLLERLEADER, MOTOR_CAN_BUS, "Lead Roller Motor");
    private final TalonFXLance intakeRollersFollower = new TalonFXLance(INTAKEROLLERFOLLOWER, MOTOR_CAN_BUS, "Follow Roller Motor");

    private final double kP = 0.7;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.01;
    private final double kV = 0.15;

    private final double retractedPosition = 0.0;
    private final double intakingPosition = 10.0; // TEST FOR TRUE VALUE ON ROBOT
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

        // Sets up motors to be inverted or not
        intakePivotMotor.setupInverted(true);
        intakeRollersMotor.setupInverted(true);

        // Sets up Coast Mode
        intakePivotMotor.setupCoastMode();
        intakeRollersMotor.setupCoastMode();
        intakeRollersFollower.setupCoastMode();

        // Sets up position
        intakePivotMotor.setPosition(0.0);
        intakeRollersMotor.setPosition(0.0);
        intakeRollersFollower.setPosition(0.0);

        // Set up Safety
        intakePivotMotor.setSafetyEnabled(false);
        intakeRollersMotor.setSafetyEnabled(false);
        intakeRollersFollower.setSafetyEnabled(false);

        // Set up PID Controllers
        intakePivotMotor.setupPIDController(0, kP, kI, kD, kS, kV, 0);
        intakeRollersMotor.setupPIDController(0, kP, kI, kD, kS, kV, 0);
        intakeRollersFollower.setupPIDController(0, kP, kI, kD, kS, kV, 0);

        // Configure the follower last so configurables above are not overwritten
        intakeRollersFollower.setupFollower(INTAKEROLLERLEADER, true);
    }

    // private void set()
    // {
    //     intakePivotMotor.set(0.2);
    //     intakeRollersMotor.set(0.2);
    // }

    /**
     * This sets the voltage of the motors.
     * @param voltage
     */
    public void setVoltage(double voltage)
    {
        intakePivotMotor.setVoltage(voltage);
        intakeRollersMotor.setVoltage(voltage);
    }

    /**
     * This sets the velocity of the motors.
     * @param velocity 
     */
    public void setVelocity(double velocity)
    {
        intakeRollersMotor.setControlVelocity(velocity);
    }

    /** 
     * This method makes the intake pick up fuel
     */
    public void pickUpFuel()
    {
        setVelocity(0.2);
        intakePivotMotor.setControlPosition(intakingPosition);
    }

    /** 
     * This method makes the intake eject fuel
     */
    public void ejectFuel()
    {
        intakePivotMotor.setControlPosition(intakingPosition);
        setVelocity(-0.2);
    }

    public void retractIntake()
    {
        intakePivotMotor.setControlPosition(retractedPosition);
        setVelocity(0.0);
    }

    public BooleanSupplier isAtPosition(double desiredPosition)
    {
        return () -> (Math.abs(intakePivotMotor.getPosition() - desiredPosition) <= 1.0);
    }

    /**
     * This method stops the intake motors
     */
    public void stop()
    {
        // intakePivotMotor.set(0.0);
        // intakeRollersMotor.set(0.0);
        setVelocity(0.0);
    }

    public Command setVelocityCommand(DoubleSupplier speed)
    {
        return run( () -> setVelocity(speed.getAsDouble()));
    }

    public Command pickupFuelCommand()
    {
        return run(() -> pickUpFuel()).withName("Pickup Fuel");
    }

    public Command ejectFuelCommand() 
    {
        return run(() -> ejectFuel()).withName("Eject Fuel");
    }

    public Command retractIntakeCommand()
    {
        return run(() -> retractIntake()).withName("Retract Intake");
    }

    public Command onCommand()
    {
        return run( () -> setVelocity(0.25) );
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
        // No idea how to get that
        return "Current Intake Velocity: " + intakeRollersMotor.getVelocity();
    }
}