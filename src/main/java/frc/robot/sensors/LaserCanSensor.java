package frc.robot.sensors;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;
import static frc.robot.Constants.LaserCan.*;

import java.lang.invoke.MethodHandles;

public class LaserCanSensor
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
    private LaserCan laserCAN = new LaserCan(ID);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new LaserCanSensor
     */
    public LaserCanSensor()
    {   
        System.out.println("  Constructor Started:  " + fullClassName);

        configureLaserCAN();
        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    private void configureLaserCAN()
    {
        try
        {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            // about width of 3.5 inches in each direction at 30in at widest ROI Would have to mount at angle
            laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 2, 16, 4));
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } 
        catch (ConfigurationFailedException e)
        {
            System.out.println("Configuration error detected:  " + e);
        }
    }
    /**
     * Returns the value of the laserCAN
    * @return lasercan.getMeasurement
    */
    public Measurement getMeasurement()
    {
        return laserCAN.getMeasurement();
    }

    /**
     * Returns the value of the laserCAN
    * @return distance from lasercan in mm
    */
    public double getDistance()
    { 
        return laserCAN.getMeasurement().distance_mm;
    }    

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "";
    }
}