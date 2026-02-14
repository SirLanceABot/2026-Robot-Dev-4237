package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Distance;

public class CANRange 
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
    private final CoreCANrange canRange; 
    private double signalStrength;
    private Distance distance;
    private boolean isDetected;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new CANrange. 
     */
    public CANRange(int CANID, double isDetectedThreshold)
    {   
        System.out.println("  Constructor Started:  " + fullClassName);
        
        canRange = new CoreCANrange(CANID, "rio");
        configCANRange(isDetectedThreshold);
        

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    private void configCANRange(double isDetectedThreshold)
    {
        CANrangeConfiguration config = new CANrangeConfiguration();
        //Lower signal strenght means longer distance, this value should be tuned to be the signal strength at the max desired distance 
        //I believe this value is arbitrary, it can be tuned using distance and signal strength values on Phoenix Tuner
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; 
        // If CANrange detects an object within x meters, it will trigger the "isDetected" signal.
        config.ProximityParams.ProximityHysteresis = 0.001;
        config.ProximityParams.ProximityThreshold = isDetectedThreshold; // in meters

        // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.
        // Short range mode has the same range as long range mode, but is more accurate for distances less than a meter
        config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; 
        
        FovParamsConfigs fov = new FovParamsConfigs();
        fov.FOVCenterX = 0; // Center of FOV in X direction in degrees (value must be within -11 and 11)
        fov.FOVCenterY = 3.5; // Center of FOV in Y direction in degrees (value must be within -11 and 11)
        fov.FOVRangeX = 27; // Range of FOV in X direction in degrees (value must be within 7 and 27)
        fov.FOVRangeY = 7; // Range of FOV in Y direction in degrees (value must be within 7 and 27)
        config = config.withFovParams(fov);


        canRange.getConfigurator().apply(config);
    }
    /**
     * Returns the value of the sensor
    * @return The value of periodData.sensorValue
    */
    /**
     * Returns the distance from the nearest object in the FOV
    * @return The value of periodData.sensorValue
    */
    public double getDistanceMeters()
    {
        return canRange.getDistance().getValue().magnitude();
    }

    public DoubleSupplier getDistanceSupplier()
    {
        return () -> getDistanceMeters();
    }

    public boolean isBallDetected()
    {
        return getDistanceMeters() * 39.3701 < 24.0; // Meters to inches
    }
    
    public BooleanSupplier isBallDetectedSupplier()
    {
        return () -> isBallDetected();
    }

    /*
     * Is the CANRange giving you a valid measurement
     */
    public boolean isCANRangeDetecting()
    {
        return canRange.getIsDetected().getValue();
    }

    public BooleanSupplier getIsDetected()
    {
        return () -> isCANRangeDetecting();
    }

    /*
     * How strong is the signal being received by the CANrange
     * Signal strength increases as distance decreases
     */
    public double getSignalStrength()
    {
        return canRange.getSignalStrength().getValue();
    }

    public DoubleSupplier getSignalStrenght()
    {
        return () -> getSignalStrength();
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "Distance: " + distance + " meters " + " Signal Strength: " + signalStrength + "  Is Detected (within 0.1m): " + isDetected;
    }
}
