package frc.robot.subsystems;

import static frc.robot.Constants.Indexigator.*;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.motors.SparkMaxLance;
import frc.robot.motors.TalonFXLance;

/**
 * Indexigator
 */
public class Indexigator extends SubsystemBase
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
    private final TalonFXLance motor = new TalonFXLance(MOTOR, MOTOR_CAN_BUS, "Indexigator Motor");
    // private final TalonFXSLance motor = new TalonFXSLance(MOTOR, MOTOR_CAN_BUS, "Indexer Motor");

    private final double kP = 0.000075;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kF = 0.000085;     // use for SparkMaxLance

    private final double INDEXIGATOR_DIAMETER_FEET = 4237.0;
    private final double GEAR_RATIO = 1.0 / 1.0;
    private final double VELOCITY_CONVERSION_FACTOR = (Math.PI * INDEXIGATOR_DIAMETER_FEET) / GEAR_RATIO; // rev/s to ft/s using gear ratio // not checked
    
    // test for TalonFXSLance
    // private final double kS = 0.0121;    // small number
    // private final double kV = 0.000084;  // change once roller is attached
    // private final double kA = 0.0;
    // private final double kG = 0.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Indexigator. 
     */
    public Indexigator()
    {
        super("Indexigator");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setSafetyEnabled(false);
        motor.setupPIDController(0, kP, kI, kD, kF);
        motor.setupCoastMode();
        // motor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR); // rev/s to ft/s
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        motor.set(speed);
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

    public double getVelocity()
    {
        return motor.getVelocity();
    }

    /**
     * Sets motor to run at a speed from 0.0 - 0.5
     * @param speed
     * @return
     */
    public Command setForwardCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

    public Command setForwardCommand()
    {
        return run( () -> set(0.4));
    }

    public Command setBackwardCommand(DoubleSupplier speed)
    {
        return run( () -> set(MathUtil.clamp(-speed.getAsDouble(), -0.5, 0.0)));
    }

    public Command setBackwardCommand()
    {
        return run( () -> set(-0.2));
    }

    // public Command setVelocityForwardCommand(DoubleSupplier speed)
    // {
    //     return run( () -> setVelocity(speed.getAsDouble()));
    // }

    // public Command setVelocityBackwardCommand(DoubleSupplier speed)
    // {
    //     return run( () -> setVelocity(-speed.getAsDouble()));
    // }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return runOnce( () -> stop() );
        // return run(this::stop);
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
