package frc.robot.subsystems;

import static frc.robot.Constants.CANbus.*;
import static frc.robot.Constants.ExampleSubsystem.MOTOR_CAN_BUS;

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
    private final SparkMaxLance acceleratorMotor = new SparkMaxLance(Constants.Accelerator.MOTOR, MOTOR_CAN_BUS, "Motor 1");

    double rollerPosition;
    double rollerVelocity;

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
        acceleratorMotor.setupFactoryDefaults();

        // motor.setupInverted(false); // Find out later

        acceleratorMotor.setupBrakeMode();
        // motor.setupCoastMode();
    }

    public double getPosition() 
    {
        return rollerPosition;
    }

    public double getVelocity() 
    {
        return rollerVelocity;
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        acceleratorMotor.set(speed);
    }

    public void stop()
    {
        acceleratorMotor.set(0.0);
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

        rollerPosition = acceleratorMotor.getPosition();

        // System.out.println("Accelerator Velocity: " + (motor.getVelocity() * 2 * (Math.PI) * (r))); // Find out radius of rods
    }

    @Override
    public String toString()
    {
        return "Current Accelerator Position: " + rollerPosition;
    }
}
