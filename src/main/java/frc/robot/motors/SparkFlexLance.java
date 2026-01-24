package frc.robot.motors;

import java.lang.invoke.MethodHandles;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkFlexLance extends MotorControllerLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    @FunctionalInterface
    private interface Function
    {
        public abstract REVLibError doAction();
    }

    private final SparkFlex motor;
    private final String motorControllerName;

    private final RelativeEncoder encoder;
    private SparkAbsoluteEncoder sparkAbsoluteEncoder = null;
    private SparkLimitSwitch forwardLimitSwitch = null;
    private SparkLimitSwitch reverseLimitSwitch = null;
    private SparkClosedLoopController sparkPIDController = null;
    private boolean useMaxMotion = false;
    
    private final ResetMode resetMode = ResetMode.kNoResetSafeParameters;
    private final PersistMode persistMode = PersistMode.kNoPersistParameters;

    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;
    private int stickyFaultCount = 0;

    /**
     * Creates a CANSparkFlex on the CANbus with a brushless motor (Neo Vortex).
     * Defaults to using the built-in encoder sensor (RelativeEncoder).
     * @param deviceId The id number of the device on the CANbus
     * @param canbus The name of the CANbus (ex. "rio" is the default name of the roboRIO CANbus)
     * @param motorControllerName The name describing the purpose of this motor controller
     */
    public SparkFlexLance(int deviceId, String canbus, String motorControllerName)
    {
        super(motorControllerName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + motorControllerName);

        this.motorControllerName = motorControllerName;

        motor = new SparkFlex(deviceId, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        sparkPIDController = motor.getClosedLoopController();

        clearStickyFaults();
        setupFactoryDefaults();
        setupFeedbackSensor();
        
        System.out.println("  Constructor Finished: " + fullClassName + " >> " + motorControllerName);
    }

    /** 
     * Check the motor controller for an error and print/log a message.
     * @param message The message to print/log (no message prints if the string is empty)
     * @param func The function to run
     */
    private void setup(Function func, String message)
    {
        REVLibError errorCode = REVLibError.kOk;
        int attemptCount = 0;
        String logMessage = "";
        
        do
        {
            errorCode = func.doAction();

            if(message != null && !message.isEmpty())
            {
                logMessage = motorControllerName + " : " + message + " " + errorCode;

                if(errorCode == REVLibError.kOk)
                    System.out.println(">> >> " + logMessage);
                else
                    DriverStation.reportWarning(logMessage, false);
                
                motorSetupPublisher.set(logMessage);
            }

            attemptCount++;
        }
        while(errorCode != REVLibError.kOk && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }

    /**
     * Setup feedback sensor to built-in encoder
     */
    private void setupFeedbackSensor()
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Feedback Sensor");
    }

    /**
     * Clear all sticky faults.
     */
    public void clearStickyFaults()
    {
        setup(() -> motor.clearFaults(), "Clear Sticky Faults");
        stickyFaultCount = 0;
    }

    /**
     * Reset to the factory defaults.
     */
    public void setupFactoryDefaults()
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        setup(() -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters), "Setup Factory Defaults");
    }

    public void setupRemoteCANCoder(int remoteSensorId)
    {
        sparkAbsoluteEncoder = motor.getAbsoluteEncoder();

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Feedback Sensor");
    }

    /**
     * Set the Periodic Frame Period.
     * <p>Defaults: Status0 - 10ms, Status1 - 20ms, Status2 - 20ms, Status3 - 50ms, Status4 - 20ms, Status5 - 200ms, Status6 - 200ms, Status7 - 250ms
     * @param frameNumber The frame number to set
     * @param periodMs The time period in milliseconds
     */
    public void setupPeriodicFramePeriod(int frameNumber, int periodMs)
    {
        motor.setControlFramePeriodMs(periodMs);
    }

    /**
     * Invert the direction of the motor controller.
     * @param isInverted True to invert the motor controller
     */
    public void setupInverted(boolean isInverted)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.inverted(isInverted);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Inverted");
    }
    
    /**
     * Sets the idle/neutral mode to brake mode.
     */
    public void setupBrakeMode()
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Brake Mode");
    }
    
    /**
     * Sets the idle/neutral mode to coast mode.
     */
    public void setupCoastMode()
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kCoast);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Coast Mode");
    }

    /**
     * Set the forward soft limit.
     * @param limit The forward soft limit value
     * @param isEnabled True to enable the forward soft limit
     */
    public void setupForwardSoftLimit(double limit, boolean isEnabled)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.softLimit.forwardSoftLimit(limit);
        motorConfig.softLimit.forwardSoftLimitEnabled(isEnabled);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Forward Soft Limit");
    }

    /**
     * Set the reverse soft limit.
     * @param limit The reverse soft limit value
     * @param isEnabled True to enable the reverse soft limit
     */
    public void setupReverseSoftLimit(double limit, boolean isEnabled)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.softLimit.reverseSoftLimit(limit);
        motorConfig.softLimit.reverseSoftLimitEnabled(isEnabled);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Reverse Soft Limit");
    }

    /**
     * Enable or disable the forward hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     * @param isNormallyOpen True if the limit switch is normally open
     */
    public void setupForwardHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        if(isNormallyOpen)
            motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        else
            motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Forward Hard Limit");
    }

    /**
     * Enable or disable the reverse hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     * @param isNormallyOpen True if the limit switch is normally open
     */
    public void setupReverseHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        if(isNormallyOpen)
            motorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        else
            motorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Reverse Hard Limit");
    }

    /**
     * Set the current limits of the motor.
     * @param currentLimit The current limit in Amps
     * @param currentThreshold The max current limit in Amps
     * @param timeThreshold The time threshold in Seconds
     */
    public void setupCurrentLimit(double currentLimit, double currentThreshold, double timeThreshold)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.smartCurrentLimit((int) currentLimit);
        motorConfig.secondaryCurrentLimit(currentThreshold, (int) (timeThreshold * 20000));
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Current Limit");
    }
    
    public double getCurrentAmps()
    {
        return motor.getOutputCurrent();
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupOpenLoopRampRate(double rampRateSeconds)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.openLoopRampRate(rampRateSeconds);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Open Loop Ramp Rate");
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupClosedLoopRampRate(double rampRateSeconds)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.closedLoopRampRate(rampRateSeconds);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Closed Loop Ramp Rate");
    }

    /**
     * Sets the voltage compensation for the motor controller. Use the battery voltage.
     * @param voltageCompensation The nominal voltage to compensate to
     */
    public void setupVoltageCompensation(double voltageCompensation)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.voltageCompensation(voltageCompensation);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Voltage Compensation");
    }

    /**
     * Set the conversion factor to convert from sensor position to mechanism position.
     * @param factor The conversion factor to multiply by
     */
    public void setupPositionConversionFactor(double factor)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        
        if(sparkAbsoluteEncoder == null)
            motorConfig.encoder.positionConversionFactor(factor);
        else
            motorConfig.absoluteEncoder.positionConversionFactor(factor);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Position Conversion Factor");
    }

    /**
     * Set the conversion factor to convert from sensor velocity to mechanism velocity.
     * @param factor The conversion factor to multiply by
     */
    public void setupVelocityConversionFactor(double factor)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();

        if(sparkAbsoluteEncoder == null)
            motorConfig.encoder.velocityConversionFactor(factor);
        else
            motorConfig.absoluteEncoder.velocityConversionFactor(factor);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Velocity Conversion Factor");
    }

    /**
     * Checks if the slotId is valid (0-3)
     * @param slotId The PID slot (0-3)
     * @return True if the slotId is valid
     */
    private boolean isValidSlotId(int slotId)
    {
        return slotId >= 0 && slotId <= 3;
    }

    /**
     * Set the PID controls for the motor.
     * @param slotId The PID slot (0-3)
     * @param kP The Proportional gain constant
     * @param kI The Integral gain constant
     * @param kD The Derivative gain constant
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD)
    {
        if(isValidSlotId(slotId))
        {
            SparkFlexConfig motorConfig = new SparkFlexConfig();
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
        
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            motorConfig.closedLoop.pid(kP, kI, kD, closedLoopSlot);
            setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup PID Controller");
        }
    }

    /**
     * Set the PID controls for the motor.
     * @param slotId The PID slot (0-3)
     * @param kP The Proportional gain constant
     * @param kI The Integral gain constant
     * @param kD The Derivative gain constant
     * @param kF The Velocity feedforward value
     */
    @SuppressWarnings("removal")
    public void setupPIDController(int slotId, double kP, double kI, double kD, double kF)
    {
        if(isValidSlotId(slotId))
        {
            SparkFlexConfig motorConfig = new SparkFlexConfig();
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            motorConfig.closedLoop.pidf(kP, kI, kD, kF, closedLoopSlot);
            setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup PID Controller");
        }
    }
    
    /**
     * Set the PID controls for the motor.
     * @param slotId The PID slot (0-3)
     * @param kP The Proportional gain constant
     * @param kI The Integral gain constant
     * @param kD The Derivative gain constant
     * @param kS Static feedforward gain
     * @param kV Velocity feedforward gain
     * @param kA Acceleration feedforward gain
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD, double kS, double kV, double kA)
    {
        if(isValidSlotId(slotId))
        {
            SparkFlexConfig motorConfig = new SparkFlexConfig();
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            motorConfig.closedLoop.pid(kP, kI, kD, closedLoopSlot)
                .feedForward.sva(kS, kV, kA, closedLoopSlot);
            setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup PID Controller");
        }
    }

    /**
     * Set the Motion Magic controls for the motor.
     * @param slotId The PID slot (0-3)
     * @param velocity The target cruise velocity (rpm)
     * @param acceleration The target acceleration (rpm / sec)
     * @param error The allowable error (rotations)
     */
    public void setupMaxMotion(int slotId, double velocity, double acceleration, double error)
    {
        if(isValidSlotId(slotId))
        {
            SparkFlexConfig motorConfig = new SparkFlexConfig();
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            motorConfig.closedLoop.maxMotion
                .cruiseVelocity(velocity)
                .maxAcceleration(acceleration)
                .allowedProfileError(error, closedLoopSlot);

            useMaxMotion = true;

            setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Max Motion");
        }
    }

    /**
     * Returns the PID values stored in the slotId
     * @param slotId The PID slot (0-3)
     * @return An array containing the PID values
     */
    public double[] getPID(int slotId)
    {
        double[] pid = {0.0, 0.0, 0.0};

        if(isValidSlotId(slotId))
        {
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;
                
            pid[0] = motor.configAccessor.closedLoop.getP(closedLoopSlot);
            pid[1] = motor.configAccessor.closedLoop.getI(closedLoopSlot);
            pid[2] = motor.configAccessor.closedLoop.getD(closedLoopSlot);
        }

        return pid;
    }
    
    /**
     * Sets a motor to be a follower of another motor.
     * Setting the power of the leader, also sets the power of the follower.
     * @param leaderId The id of the leader motor on the can bus
     * @param isInverted True to invert the motor so it runs opposite of the leader
     */
    public void setupFollower(int leaderId, boolean isInverted)
    {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.follow(leaderId, isInverted);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Follower");
    }

    /**
     * @return true if limit switch is pressed
     */
    public boolean isForwardLimitSwitchPressed()
    {
        return forwardLimitSwitch.isPressed();
    }

    /**
     * @return true if limit switch is pressed
     */
    public boolean isReverseLimitSwitchPressed()
    {
        return reverseLimitSwitch.isPressed();
    }

    private void checkFault(boolean check, String msg)
    {
        if(check)
        {
            motorSetupPublisher.set(motorControllerName + " : " + msg);
            stickyFaultCount++;
        }
    }

    /**
     * Logs and then clears the sticky faults
     */
    public void logStickyFaults()
    {
        Faults faults = motor.getStickyFaults();
        Warnings warnings = motor.getStickyWarnings();

        if(setupErrorCount > 0)
        {
            motorSetupPublisher.set(motorControllerName + " : " + setupErrorCount + " setup errors");
        }

        checkFault(faults.can, "Fault - CAN");
        checkFault(faults.escEeprom, "Fault - Esc Eeprom");
        checkFault(faults.firmware, "Fault - Firmware");
        checkFault(faults.gateDriver, "Fault - Gate Driver");
        checkFault(faults.motorType, "Fault - Motor Type");
        checkFault(faults.other, "Fault - Other");
        checkFault(faults.sensor, "Fault - Sensor");
        checkFault(faults.temperature, "Fault - Temperature");
        checkFault(warnings.brownout, "Warning - Brownout");
        checkFault(warnings.escEeprom, "Warning - Esc Eeprom");
        checkFault(warnings.extEeprom, "Warning - Ext Eeprom");
        checkFault(warnings.hasReset, "Warning - Has Reset");
        checkFault(warnings.other, "Warning - Other");
        checkFault(warnings.overcurrent, "Warning - Overcurrent");
        checkFault(warnings.sensor, "Warning - Sensor");
        checkFault(warnings.stall, "Warning - Stall");
        
        // if(faults.can)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Fault - CAN");
        //     faultsCount++;
        // }
        // if(faults.sensor)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Fault - Sensor");
        //     faultsCount++;
        // }
        // if(faults.temperature)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Fault - Temperature");
        //     faultsCount++;
        // }

        // if(warnings.brownout)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Warning - Brownout");
        //     faultsCount++;
        // }
        // if(warnings.hasReset)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Warning - Has Reset");
        //     faultsCount++;
        // }
        // if(warnings.overcurrent)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Warning - Overcurrent");
        //     faultsCount++;
        // }
        // if(warnings.stall)
        // {
        //     motorFaultsPublisher.set(motorControllerName + " : Warning - Stall");
        //     faultsCount++;
        // }

        if(stickyFaultCount == 0)
        {
            motorFaultsPublisher.set(motorControllerName + " : No Sticky Faults");
        }

        clearStickyFaults();
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * Uses PID slot 0 by default.
     * @param position The position to move the motor to
     */
    public void setControlPosition(double position)
    {
        setControlPosition(position, 0);
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position to move the motor to
     * @param slotId The PID slot (0-3)
     */
    public void setControlPosition(double position, int slotId)
    {
        if(isValidSlotId(slotId))
        {
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            if(!useMaxMotion)
                sparkPIDController.setSetpoint(position, ControlType.kPosition, closedLoopSlot);
            else
                sparkPIDController.setSetpoint(position, ControlType.kMAXMotionPositionControl, closedLoopSlot);
        }
    }

    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * Uses PID slot 0 by default.
     * @param velocity The velocity to spin the motor at
     */
    public void setControlVelocity(double velocity)
    {
        setControlVelocity(velocity, 0);
    }

    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param velocity The velocity to spin the motor at
     * @param slotId The PID slot (0-3)
     */
    public void setControlVelocity(double velocity, int slotId)
    {
        if(isValidSlotId(slotId))
        {
            ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            if(!useMaxMotion)
                sparkPIDController.setSetpoint(velocity, ControlType.kVelocity, closedLoopSlot);
            else
                sparkPIDController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, closedLoopSlot);
        }
    }

    /**
     * Set the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position of the encoder
     */
    public void setPosition(double position)
    {
        encoder.setPosition(position);
    }

    /**
     * Get the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @return The position of the encoder
     */
    public double getPosition()
    {
        if(sparkAbsoluteEncoder == null)
            return encoder.getPosition();
        else
            return sparkAbsoluteEncoder.getPosition();
    }

    /**
     * Get the velocity of the encoder.
     * Units are RPMs by default, but can be changed using the conversion factor.
     * @return The velocity of the encoder
     */    
    public double getVelocity()
    {
        if(sparkAbsoluteEncoder == null)
            return encoder.getVelocity();
        else
            return sparkAbsoluteEncoder.getVelocity();
    }

    /**
     * Get the applied motor voltage (in volts).
     * @return The voltage
     */    
    public double getMotorVoltage()
    {
        return motor.getBusVoltage();
    }

    @Override
    public void stopMotor()
    {
        set(0.0);
    }

    @Override
    public String getDescription()
    {
        return motorControllerName;
    }

    @Override
    public void set(double speed)
    {
        motor.set(speed);
        feed();
    }

    @Override
    public void setVoltage(double outputVolts) 
    {
        motor.setVoltage(outputVolts);
        feed();
    }

    @Override
    public double get()
    {
        return motor.get();
    }

    @Override
    public boolean getInverted()
    {
        return motor.configAccessor.getInverted();
    }

    @Override
    public void disable()
    {
        motor.disable();
    }
}
