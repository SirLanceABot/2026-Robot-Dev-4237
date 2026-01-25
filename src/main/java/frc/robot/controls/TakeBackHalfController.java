package frc.robot.controls;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public final class TakeBackHalfController implements Sendable
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private static int instances;

    private static final double defaultGain = 1.e-5;
    private double gain;
    
    private static final double defaultTolerance = 0.05;
    private double tolerance;

    private static final double defaultSetpoint = 0.0;
    private double setpoint;

    private double measurement = 0.0;

    private static final double defaultFeedForward = 0.1;

    private double prev_error;
    private double output;
    private double feedForward;
    private double tbh;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * creates a new Take-Back-Half controller.
     * 
     * @param gain to correct error
     * @param tolerance position error which is tolerable
     */
    public TakeBackHalfController(double gain, double tolerance)
    {
        System.out.println(" Constructor Stated: " + fullClassName);

        instances++;

        setGain(gain);
        setTolerance(tolerance);
        setSetpoint(defaultSetpoint);

        SendableRegistry.addLW(this, "TBHController", instances);

        System.out.println(" Contructor Finished: " + fullClassName);
    }


    /**
     * creates a new Take-Back-Half controller.
     * using default gain and tolerance
     */
    public TakeBackHalfController()
    {
        this(defaultGain, defaultTolerance);
    }

    public void setGain(double gain)
    {
        this.gain = gain;
    }

    public double getGain()
    {
        return gain;
    }

    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public double getTolerance()
    {
        return tolerance;
    }

    /**
     * 
     * @param setpoint the desired setpoint (default setpoint is 0.0)
     * @param feedForward approximate control signal needed for the setpoint used to initiate ramp-up to speed
     * 0 gives slower ramp to speed with less overshoot. 1 gives fastest arrival at speed and likely more overshoot
     */
    public void setSetpoint(double setpoint, double feedForward)
    {
        this.setpoint = setpoint;
        this.feedForward = feedForward;
        prev_error = 1.0;
        output = 1.0;
        tbh = 2.0 * feedForward - 1;
    }

    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
        setSetpoint(setpoint, defaultFeedForward);
    }

    public double getSetpoint()
    {
        return setpoint;
    }

    public boolean atSetpoint()
    {
        return Math.abs(setpoint - measurement) < tolerance*setpoint;
    }

    public double getMeasurement()
    {
        return measurement;
    }

    public double getError()
    {
        return setpoint - measurement;
    }

    /**
     * returns calculated control output
     * 
     * @param measurement the most recent measurement of the process variable
     * @return calculated motor input (conroller output)
     */
    public double calculate(double measurement)
    {
        // System.out.println("*****Measurement = " + measurement);
        // System.out.println("*****Setpoint = " + setpoint);

        if(setpoint == 0.0) 
            return 0.0;

        this.measurement = measurement;

        var error = setpoint - measurement;
        output += gain * error;
        // System.out.println("*****Error = " + error);

        if(error >= 0 != prev_error >= 0)
        {
            output = 0.5 * (output + tbh);
            tbh = output;
            prev_error = error;
        }

        // System.out.println("*****Calculated Output = " + output);

        return MathUtil.clamp(output, 0.0, 1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("TBHController");
        builder.addDoubleProperty("gain", this::getGain, this::setGain);
        builder.addDoubleProperty("tolerance", this::getTolerance, this::setTolerance);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("measurement", this::getMeasurement, null);
        builder.addDoubleProperty("error", this::getError, null);
        builder.addBooleanProperty("atSetpoint", this::atSetpoint, null);
    }
}
