package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import static frc.robot.Constants.Hopper.*;

/**
 * Use this class as a template to create other sensors.
 */
public class Hopper
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

    private CANRange canRangeRight = new CANRange(CAN_RANGE_RIGHT, 3.0);
    private CANRange canRangeLeft = new CANRange(CAN_RANGE_LEFT, 3.0);
    // private final DigitalInput limitSwitchRight = new DigitalInput(LIMIT_SWITCH_RIGHT);
    // private final DigitalInput limitSwitchLeft = new DigitalInput(LIMIT_SWITCH_LEFT);
    private Debouncer debouncer = new Debouncer(0.5);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Hopper()
    {   
        System.out.println("  Constructor Started:  " + fullClassName);


        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    /**
     * Returns the value of the sensor
    * @return The value of periodData.sensorValue
    */
    // public Boolean getLimitSwitchRight()
    // {
    //     return limitSwitchRight.get();
    // }
    
    // public Boolean getLimitSwitchLeft()
    // {
    //     return limitSwitchLeft.get();
    // }

    // public BooleanSupplier isHopperClosed()
    // {
    //     return () -> (getLimitSwitchRight() && getLimitSwitchLeft());
    // }
    public DoubleSupplier getRightCanRangeDistance()
    {
        return () -> canRangeRight.getDistanceMeters();
    }

    public DoubleSupplier getLeftCanRangeDistance()
    {
        return () -> canRangeLeft.getDistanceMeters();
    }

    public BooleanSupplier isRightFullSupplier()
    {
        return () -> debouncer.calculate(canRangeRight.isBallDetected(HOPPER_EXTENDED_LENGTH));
    }

    public BooleanSupplier isLeftFullSupplier()
    {
        return () -> debouncer.calculate(canRangeLeft.isBallDetected(HOPPER_EXTENDED_LENGTH));
    }

    public BooleanSupplier isHopperFullSupplier()
    {
        return () -> debouncer.calculate(canRangeRight.isBallDetected(HOPPER_EXTENDED_LENGTH) && canRangeLeft.isBallDetected(HOPPER_EXTENDED_LENGTH));
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "";
    }
}