package frc.robot.subsystems;

import static frc.robot.Constants.Agitator.MOTOR;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.SparkMaxLance;

/**
 * Creates an agitator
 */
public class Agitator extends SubsystemBase
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
    private final SparkMaxLance agitatorMotor = new SparkMaxLance(MOTOR, Constants.Agitator.MOTOR_CAN_BUS, "");
    // private final SparkMaxLance agitatorMotor = new SparkMaxLance(3, "ROBORIO", "Agitator");


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new agitator. 
     */
    public Agitator()
    {
        super("Agitator Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        agitatorMotor.setupFactoryDefaults();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        agitatorMotor.set(speed);
    }

    public void stop()
    {
        set(0.0);
    }

    public Command forwardCommand()
    {
        return run( () -> set(0.25) );
    }

    public Command reverseCommand()
    {
        return run( () -> set(-0.25) );
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
    }

    @Override
    public String toString()
    {
        return "";
    }
}
