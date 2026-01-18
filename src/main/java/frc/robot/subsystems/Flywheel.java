package frc.robot.subsystems;

import static frc.robot.Constants.CANbus.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Flywheel extends SubsystemBase
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
    private final TalonFXLance leadMotor = new TalonFXLance(Constants.Flywheel.LEADMOTOR, ROBORIO, "Motor 1");
    private final TalonFXLance followMotor = new TalonFXLance(Constants.Flywheel.FOLLOWMOTOR, ROBORIO, "Motor 2");

    // PID constants
    private final double kP = 0.3;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.0;
    private final double kV = 0.0;

    private final double velocityConversionFactor = 1.0; // figure out units

    InterpolatingDoubleTreeMap scoreMap = new InterpolatingDoubleTreeMap();
    // InterpolatingDoubleTreeMap passLeftMap = new InterpolatingDoubleTreeMap();
    // InterpolatingDoubleTreeMap passRightMap = new InterpolatingDoubleTreeMap();


    // private final PIDController flywheelPID = new PIDController(0, 0, 0);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Shooter. 
     */
    public Flywheel()
    {
        super("Flywheel");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();
        configScoreMap();
        // configLeftPassMap();
        // configRightPassMap();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        leadMotor.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();

        followMotor.setupFollower(1, false);

        leadMotor.setSafetyEnabled(true);
        followMotor.setSafetyEnabled(true);

        leadMotor.setupBrakeMode();
        followMotor.setupBrakeMode();

        leadMotor.setupPIDController(0, kP, kI, kD, kS, kV, 0.0, 0.0);
        
        leadMotor.setupVelocityConversionFactor(velocityConversionFactor);
    }

    private void configScoreMap()
    {
        // first value is distance from the hub (in alliance zone), second is motor power
        // not tested values
        scoreMap.put(0.0, 4237.0);
        scoreMap.put(1.0, 4237.0);
        scoreMap.put(2.0, 4237.0);
        scoreMap.put(3.0, 4237.0);
        scoreMap.put(4.0, 4237.0);
        scoreMap.put(5.0, 4237.0);
        scoreMap.put(6.0, 4237.0);
        scoreMap.put(7.0, 4237.0);
        scoreMap.put(8.0, 4237.0);
        scoreMap.put(9.0, 4237.0);
        scoreMap.put(10.0, 4237.0);
        scoreMap.put(11.0, 4237.0);
        scoreMap.put(12.0, 4237.0);
        scoreMap.put(13.0, 4237.0);
        scoreMap.put(14.0, 4237.0);
    }

    // idk if we need these maps or not
    // private void configLeftPassMap()
    // {}

    // private void configRightPassMap()
    // {}

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        leadMotor.set(speed);
        // followMotor.set(speed);
    }
    
    private void setControlVelocity(double speed)
    {
        leadMotor.setControlVelocity(speed);
        // followMotor.setControlVelocity(speed);
    }

    private void stop()
    {
        set(0.0);
    }

    private void shoot(double speed)
    {
        setControlVelocity(speed);
    }

    public double getVelocity()
    {
        return leadMotor.getVelocity();
    }

    // private double calcScoreVelocityFromDistance()
    // {}

    // more than one passing location?
    // private double calcPassVelocityFromDistance()
    // {}

    public Command onCommand()
    {
        return run( () -> set(0.1) );
    }

    public Command shootCommand(DoubleSupplier speed)
    {
        return run( () -> shoot(speed.getAsDouble()));
    }

    public BooleanSupplier isAtSetSpeed(double targetSpeed, double speedTolerance)
    {
        double currentSpeed = leadMotor.getVelocity();
        
        return () ->
        {
        if((currentSpeed + speedTolerance > targetSpeed) && (currentSpeed - speedTolerance < targetSpeed))
        {
            return true;
        }
        else
        {
            return false;
        }
        };
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return runOnce( () -> stop());
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
        return "Flywheel Velocity = " + getVelocity();
    }
}
