package frc.robot.subsystems;

import static frc.robot.Constants.Accelerator.*;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.SparkMaxLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Accelerator extends SubsystemBase
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
    private final SparkMaxLance motor = new SparkMaxLance(MOTOR, MOTOR_CAN_BUS, "Accelerator Motor");

    // private final SparkMaxLance motor2 = new SparkMaxLance(1, MOTOR_CAN_BUS, "Follower Motor");

    private final double ACCELERATOR_DIAMETER = 4237.0;
    private final double GEAR_RATIO = 1.0 / 1.0;
    private final double VELOCITY_CONVERSION_FACTOR = (Math.PI * ACCELERATOR_DIAMETER) / GEAR_RATIO; // rev/s to ft/s using gear ratio // not checked

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Accelerator()
    {
        super("Accelerator");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setupInverted(true); // Find out later
        // motor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR); // rev/s to ft/s

        motor.setSafetyEnabled(false);

        motor.setPosition(0);

        motor.setupForwardHardLimitSwitch(true, true);
        motor.setupReverseHardLimitSwitch(true, true);

        motor.setupForwardSoftLimit(100, false);
        motor.setupReverseSoftLimit(0, false);

        // motor.setupMaxMotion(600.0, 6.0, 0.0, 0);

        // motor.setupCurrentLimit(5.0, 45.0, 0.5);
        //Current Threshold depends on speed sent to motor

        motor.setupPIDController(0, 0.0001, 0, 0);
        // motor.setupCoastMode();
        motor.setupBrakeMode();

        // motor2.setupFactoryDefaults();
        // motor2.setupBrakeMode();

        // motor2.setupFollower(MOTOR, false);
    }

    public double getPosition() 
    {
        return motor.getPosition();
    }

    public double getVelocity() 
    {
        return motor.getVelocity();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        motor.set(speed);
    }

    private void setControlVelocity(double velocity)
    {
        motor.setControlVelocity(velocity);
    }

    private void setControlPosition(double position)
    {
        motor.setControlPosition(position);
    }

    public void stop()
    {
        set(0.0);
    }

    public Command onCommand()
    {
        // return run( () -> set(0.25) );
        return run( () -> set(0.10) );
    }

    public Command reverseCommand()
    {
        // return run( () -> set(-0.25) );
        return run( () -> set(-0.10) );
    }

    public Command feedToShooterCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

    public Command reverseCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), -0.5, 0.0)) );
    }

    public Command setVelocityCommand(double targetSpeed)
    {
        return run( () -> setControlVelocity(targetSpeed));
    }

    public Command setPositionCommand(double targetPosition)
    {
        return run( () -> setControlPosition(targetPosition));
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        // return run( () -> stop() );
        return run(this::stop);
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        // System.out.println("Accelerator Velocity: " + (motor.getVelocity() * 2 * (Math.PI) * (r))); // Find out radius of rods
    }

    @Override
    public String toString()
    {
        return "Accelerator Current Velocity: " + getVelocity();
    }
}
