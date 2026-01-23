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
        // motor.setupInverted(false); // Find out later
        // motor.setupVelocityConversionFactor();

        motor.setupPIDController(0, 0.1, 0.001, 2);
        motor.setupCoastMode();
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

    public void stop()
    {
        set(0.0);
    }

    public Command onCommand()
    {
        return run( () -> set(0.25) );
    }

    public Command reverseCommand()
    {
        return run( () -> set(-0.25) );
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
        return "Current Accelerator Position: " + getPosition();
    }
}
