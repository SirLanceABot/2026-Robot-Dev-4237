package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
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

    private CANRange canRangeRight;
    private CANRange canRangeLeft;
    private final DigitalInput limitSwitchRight = new DigitalInput(LIMIT_SWITCH_RIGHT);
    private final DigitalInput limitSwitchLeft = new DigitalInput(LIMIT_SWITCH_LEFT);
    private Debouncer debouncer = new Debouncer(0.5);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Hopper(CANRange canRangeRight, CANRange canRangeLeft)
    {   
        System.out.println("  Constructor Started:  " + fullClassName);

        this.canRangeRight = canRangeRight;
        this.canRangeLeft = canRangeLeft;

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    /**
     * Returns the value of the sensor
    * @return The value of periodData.sensorValue
    */
    public BooleanSupplier getLimitSwitch1Supplier()
    {
        return () -> limitSwitchRight.get();
    }
    
    public BooleanSupplier getLimitSwitch2Supplier()
    {
        return () -> limitSwitchLeft.get();
    }

    public BooleanSupplier isHopperFullSupplier()
    {
        return () -> debouncer.calculate(canRangeRight.isBallDetected(HOPPER_EXTENDED_LENGTH) && canRangeLeft.isBallDetected(HOPPER_EXTENDED_LENGTH));
    }

    public BooleanSupplier isHopperClosed()
    {
        return () -> (getLimitSwitch1Supplier().getAsBoolean() && getLimitSwitch2Supplier().getAsBoolean());
    }

    

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "";
    }
}