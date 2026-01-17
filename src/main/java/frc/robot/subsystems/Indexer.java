package frc.robot.subsystems;

import static frc.robot.Constants.CANbus.*;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.SparkMaxLance;
import frc.robot.motors.TalonFXLance;

/**
 * Indexer
 */
public class Indexer extends SubsystemBase
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
    private final SparkMaxLance motor = new SparkMaxLance(3, ROBORIO, "Motor");
    // private final TalonFXS motor2 = new TalonFXS(0, ROBORIO, "motor 2");

    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.0;  // small number
    private final double kV = 1000; // change once roller is attached
    private final double kA = 0.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Indexer. 
     */
    public Indexer()
    {
        super("Indexer");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setSafetyEnabled(true);
        motor.setupPIDController(0, kP, kI, kD);
        motor.setupCoastMode();

        // motor2.setupFactoryDefaults();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        motor.set(speed);
        motor.feed();
        
        // motor2.set(speed);
        // motor2.feed();
    }

    private void setVelocity(double speed)
    {
        motor.setControlVelocity(speed);
        // add conversion factor

        // motor2.setControlVelocity(speed);
    }

    public void stop()
    {
        set(0.0);
    }

    public Command onCommand()
    {
        return run( () -> set(0.1) );
    }

    /**
     * Sets motor to run at a speed from 0.0 - 0.5
     * @param speed
     * @return
     */
    public Command setCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

    public Command setVelocityCommand(DoubleSupplier speed)
    {
        return run( () -> setVelocity(speed.getAsDouble()));
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
