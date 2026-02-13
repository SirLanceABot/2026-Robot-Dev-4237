package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Use this class as a template to create other sensors.
 */
public class RangerDistanceSensor
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
    private static final double k15DegreeFOVVoltageToDistanceConversionFactor = 32.50930976; // Y-int= -2.695384202
    // 0-1 Short range (2-40in / 50-1000mm recommended) - 15Hz
    private static final double k20DegreeFOVVoltageToDistanceConversionFactor = 48.78136376; // Y-int= -4.985354503
    // 0-0 Moderate range (2-60in / 50-1500mm recommended) - 10Hz
    private static final double k27DegreeFOVVoltageToDistanceConversionFactor = 76.85612461; // Y-int= -9.925949725
    // 1-0 Long range (2-120in / 50-3000mm recommended) - 2.5Hz


    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final AnalogInput swyftDistanceSensor = new AnalogInput(0);
    private final DigitalInput swyftDetectorSensor = new DigitalInput(0);
    private double distance_in;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public RangerDistanceSensor()
    {   
        System.out.println("  Constructor Started:  " + fullClassName);


        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    public void setDistance_in(double distance_in)
    {
        this.distance_in = distance_in;
    }

    public void getDistanceInches15DegreeFOV(double distance_in)
    {
        double voltage = swyftDistanceSensor.getAverageVoltage();
        setDistance_in((voltage * k15DegreeFOVVoltageToDistanceConversionFactor)-2.695384202);
    }

    public void getDistanceInches20DegreeFOV(double distance_in)
    {
        double voltage = swyftDistanceSensor.getAverageVoltage();
        setDistance_in((voltage * k20DegreeFOVVoltageToDistanceConversionFactor)-4.985354503);
    }

    public void getDistanceInches27DegreeFOV(double distance_in)
    {
        double voltage = swyftDistanceSensor.getAverageVoltage();
        setDistance_in((voltage * k27DegreeFOVVoltageToDistanceConversionFactor)-9.925949725);
    }

    public double getDistanceMM()
    {
        return distance_in * 25.4;
    }

    public boolean isObjectDetected()
    {
        return swyftDetectorSensor.get();
    }

    /**
     * Returns the value of the Ranger Distance Sensore
    * @return The value of periodData.sensorValue
    */
    

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "Super duper cool awsome Swyft distance sensor";
    }
}