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
    private final TalonFXLance motor1 = new TalonFXLance(1, ROBORIO, "Motor 1");
    private final TalonFXLance motor2 = new TalonFXLance(2, ROBORIO, "Motor 2");
    private final TalonFXLance followMotor = new TalonFXLance(3, ROBORIO, "Motor 3");
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
        // FACTORY DEFAULTS 

        motor1.setupFactoryDefaults();
        motor2.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();

        // Sets up motors to be inverted
        motor1.setupInverted(true);
        motor2.setupInverted(true);
        followMotor.setupInverted(true);

        // Sets up Brake Mode
        motor1.setupBrakeMode();
        motor2.setupBrakeMode();
        followMotor.setupBrakeMode();

        //Sets up position
        motor1.setPosition(0.0);
        motor2.setPosition(0.0);
        followMotor.setPosition(0.0);

        // Set up Safety
        motor1.setSafetyEnabled(false);
        motor2.setSafetyEnabled(false);
        followMotor.setSafetyEnabled(false);
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    public void set(double speed)
    {
        motor1.set(speed);
        motor2.set(speed);
        followMotor.set(speed);
    }

    public void setVoltage(double voltage)
    {
        motor1.setVoltage(voltage);
        motor2.setVoltage(voltage);
        followMotor.setVoltage(voltage);
    }

    public void pickUpFuel()
    {
        motor1.setupCoastMode();
        motor2.setupCoastMode();
        followMotor.setupCoastMode();
        set(-0.2);
    }

    public void ejectFuel()
    {
       motor1.setupCoastMode();
       motor2.setupCoastMode();
       followMotor.setupCoastMode();
       set(0.2);
    }
    public void stop()
    {
        motor1.set(0.0);
        motor2.set(0.0);
        followMotor.set(0.0);
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
