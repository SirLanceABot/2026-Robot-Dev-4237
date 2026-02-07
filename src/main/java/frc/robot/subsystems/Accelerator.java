package frc.robot.subsystems;

import static frc.robot.Constants.Accelerator.*;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.SparkMaxLance;
import frc.robot.motors.TalonFXLance;

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
    private final SparkMaxLance leadMotor = new SparkMaxLance(MOTOR, MOTOR_CAN_BUS, "Lead Motor"); // Neo550
    // private final TalonFXLance followerMotor1 = new TalonFXLance(1, MOTOR_CAN_BUS, "Follower Motor 1"); // Kracken
    // private final TalonFXLance followerMotor2 = new TalonFXLance(2, MOTOR_CAN_BUS, "Follower Motor 2"); // Kracken

    private final double ACCELERATOR_DIAMETER = 0.1875;
    // Neo 550:
    private final double GEAR_RATIO = 12.0 / 1.0;
    private final double LEAD_MOTOR_VELOCITY_CONVERSION_FACTOR = ((Math.PI * ACCELERATOR_DIAMETER) / GEAR_RATIO) / 60; // rev/m to ft/s using gear ratio

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
        leadMotor.setupFactoryDefaults();
        leadMotor.setupInverted(false); // isn't inverted on current shooter
        // leadMotor.setupVelocityConversionFactor(LEAD_MOTOR_VELOCITY_CONVERSION_FACTOR); // rev/m to ft/s

        leadMotor.setSafetyEnabled(false);

        leadMotor.setPosition(0);

        // motor.setupForwardHardLimitSwitch(false, true);
        // motor.setupReverseHardLimitSwitch(false, true);

        // motor.setupForwardSoftLimit(100, false);
        // motor.setupReverseSoftLimit(0, false);

        // motor.setupMaxMotion(1200.0, 400.0, 0.05, 0);
        // motor.setupPIDController(0, 0.7, 0.0, 0.0, 0.0001);

        // motor.setupCurrentLimit(5.0, 45.0, 0.5);
        //Current Threshold depends on speed sent to motor

        // both value sets of values currently work (not tuned) on rev/sec - NOT TESTED with the conversion factor
        leadMotor.setupPIDController(0, 0.00002, 0.0, 0.0, 0.00103);
        // leadMotor.setupPIDController(0, 0.00003, 0.0, 0, 0.15 , 0.001, 0.0); // kI used to be 0.0000002
        
        
        leadMotor.setupCoastMode();
        // motor.setupBrakeMode();

        // followerMotor.setupFactoryDefaults();
        // followerMotor.setupBrakeMode();

        // followerMotor.setupFollower(MOTOR, false);
    }

    public double getPosition() 
    {
        return leadMotor.getPosition();
    }

    public double getVelocity() 
    {
        return leadMotor.getVelocity();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        leadMotor.set(speed);
    }

    private void setControlVelocity(double velocity)
    {
        leadMotor.setControlVelocity(velocity);
    }

    private void setControlPosition(double position)
    {
        leadMotor.setControlPosition(position);
    }

    public void stop()
    {
        set(0.0);
    }

    public Command onCommand()
    {
        // return run( () -> set(0.25) );
        return run( () -> set(.2) );
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
        return runOnce( () -> setControlPosition(targetPosition));
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        // return run( () -> stop() );
        return runOnce(this::stop);
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
        return "Accelerator Velocity: " + getVelocity();
    }
}
