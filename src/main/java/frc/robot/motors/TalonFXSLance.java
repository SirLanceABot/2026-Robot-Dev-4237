package frc.robot.motors;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.DriverStation;

public class TalonFXSLance extends MotorControllerLance
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
        public abstract StatusCode doAction();
    }

    @FunctionalInterface
    private interface StatusSignalFunction
    {
        public abstract StatusSignal<Boolean> doAction();
    }

    private final TalonFXS motor;
    private final TalonFXSConfiguration motorConfigs;
    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;
    private final String motorControllerName;

    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;
    private int stickyFaultCount = 0;

    /**
     * Creates a TalonFX on the CANbus with a brushless motor (Falcon500 or Kraken).
     * Defaults to using the built-in encoder sensor (RotorSensor).
     * @param deviceId The id number of the device on the CANbus
     * @param canbus The name of the CANbus (ex. "rio" is the default name of the roboRIO CANbus)
     * @param motorControllerName The name describing the purpose of this motor controller
     */
    public TalonFXSLance(int deviceId, String canbus, String motorControllerName)
    {
        super(motorControllerName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + motorControllerName);

        this.motorControllerName = motorControllerName;

        motor = new TalonFXS(deviceId, new CANBus(canbus));
        motorConfigs = new TalonFXSConfiguration();
        positionVoltage = new PositionVoltage(0.0);
        velocityVoltage = new VelocityVoltage(0);

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
        StatusCode errorCode = StatusCode.OK;
        int attemptCount = 0;
        String logMessage = "";
        
        do
        {
            errorCode = func.doAction();

            if(message != null && !message.isEmpty())
            {
                logMessage = motorControllerName + " : " + message + " " + errorCode;

                if(errorCode == StatusCode.OK)
                    System.out.println(">> >> " + logMessage);
                else
                    DriverStation.reportWarning(logMessage, true);

                motorSetupPublisher.set(logMessage);
            }

            attemptCount++;
        }
        while(errorCode != StatusCode.OK && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }

    /**
     * Setup feedback sensor to built-in encoder
     */
    private void setupFeedbackSensor()
    {
        motorConfigs.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
        setup(() -> motor.getConfigurator().apply(motorConfigs.ExternalFeedback), "Setup Feedback Sensor");
    }

    /**
     * Clear all sticky faults.
     */
    public void clearStickyFaults()
    {
        setup(() -> motor.clearStickyFaults(), "Clear Sticky Faults");
        stickyFaultCount = 0;
    }

    /**
     * Reset to the factory defaults.
     */
    public void setupFactoryDefaults()
    {
        // Create a new TalonFXS Configuration, which contains factory defaults
        TalonFXSConfiguration factoryConfigs = new TalonFXSConfiguration();
        factoryConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Apply a new configuration to the motor to reset to factory defaults
        setup(() -> motor.getConfigurator().apply(factoryConfigs), "Setup Factory Defaults");

        // Update the motorConfigs with the new motor configuration (factory defaults)
        setup(() -> motor.getConfigurator().refresh(motorConfigs), "");
    }

    public void setupRemoteCANCoder(int remoteSensorId)
    {
        motorConfigs.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        motorConfigs.ExternalFeedback.FeedbackRemoteSensorID = remoteSensorId;
        setup(() -> motor.getConfigurator().apply(motorConfigs.ExternalFeedback), "Setup Remote CANCoder");
    }

    /**
     * Set the Periodic Frame Period.
     * @param frameNumber The frame number to set
     * @param periodMs The time period in milliseconds
     */
    public void setupPeriodicFramePeriod(int frameNumber, int periodMs)
    {
        // FIXME
    }

    /**
     * Invert the direction of the motor controller.
     * @param isInverted True to invert the motor controller
     */
    public void setupInverted(boolean isInverted)
    {
        if(isInverted)
            motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        setup(() -> motor.getConfigurator().apply(motorConfigs.MotorOutput), "Setup Inverted");
    }

    /**
     * Sets the idle/neutral mode to brake mode.
     */
    public void setupBrakeMode()
    {
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        setup(() -> motor.getConfigurator().apply(motorConfigs.MotorOutput), "Setup Brake Mode");
    }

    /**
     * Sets the idle/neutral mode to coast mode.
     */
    public void setupCoastMode()
    {
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        setup(() -> motor.getConfigurator().apply(motorConfigs.MotorOutput), "Setup Coast Mode");
    }

    /**
     * Set the forward soft limit.
     * @param limit The forward soft limit value
     * @param isEnabled True to enable the forward soft limit
     */
    public void setupForwardSoftLimit(double limit, boolean isEnabled)
    {
        motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit;
        motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(motorConfigs.SoftwareLimitSwitch), "Setup Forward Soft Limit");
    }

    /**
     * Set the reverse soft limit.
     * @param limit The reverse soft limit value
     * @param isEnabled True to enable the reverse soft limit
     */
    public void setupReverseSoftLimit(double limit, boolean isEnabled)
    {
        motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limit;
        motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(motorConfigs.SoftwareLimitSwitch), "Setup Reverse Soft Limit");
    }

    /**
     * Enable or disable the forward hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     * @param isNormallyOpen True if the limit switch is normally open
     */
    public void setupForwardHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        motorConfigs.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;

        if(isNormallyOpen)
            motorConfigs.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        else
            motorConfigs.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(motorConfigs.HardwareLimitSwitch), "Setup Forward Hard Limit");
    }

    /**
     * Enable or disable the reverse hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     * @param isNormallyOpen True if the limit switch is normally open
     */
    public void setupReverseHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        motorConfigs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        if(isNormallyOpen)
            motorConfigs.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        else
            motorConfigs.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
        motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(motorConfigs.HardwareLimitSwitch), "Setup Reverse Hard Limit");
    }

    /**
     * Set the current limits of the motor.
     * @param currentLimit The current limit in Amps
     * @param currentThreshold The max current limit in Amps
     * @param timeThreshold The time threshold in Seconds
     */
    public void setupCurrentLimit(double currentLimit, double currentThreshold, double timeThreshold)
    {
        motorConfigs.CurrentLimits.SupplyCurrentLimit = currentThreshold;
        motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        motorConfigs.CurrentLimits.SupplyCurrentLowerTime = timeThreshold;
        motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        setup(() -> motor.getConfigurator().apply(motorConfigs.CurrentLimits), "Setup Current Limit");
    }

    public double getCurrentAmps()
    {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupOpenLoopRampRate(double rampRateSeconds)
    {
        // motorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRateSeconds;
        // motorConfigs.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRateSeconds;

        motorConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRateSeconds;
        setup(() -> motor.getConfigurator().apply(motorConfigs.OpenLoopRamps), "Setup Open Loop Ramp Rate");
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupClosedLoopRampRate(double rampRateSeconds)
    {
        // motorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRateSeconds;
        // motorConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRateSeconds;

        motorConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRateSeconds;
        setup(() -> motor.getConfigurator().apply(motorConfigs.ClosedLoopRamps), "Setup Closed Loop Ramp Rate");
    }

    /**
     * Sets the voltage compensation for the motor controller. Use the battery voltage.
     * @param voltageCompensation The nominal voltage to compensate to
     */
    public void setupVoltageCompensation(double voltageCompensation)
    {
        motorConfigs.Voltage.PeakForwardVoltage = voltageCompensation;
        motorConfigs.Voltage.PeakReverseVoltage = -voltageCompensation;
        setup(() -> motor.getConfigurator().apply(motorConfigs.Voltage), "Setup Voltage Compensation");
    }

    /**
     * Set the conversion factor to convert from sensor rotations to mechanism output.
     * @param factor The conversion factor to multiply by
     */
    public void setupPositionConversionFactor(double factor)
    {
        motorConfigs.ExternalFeedback.SensorToMechanismRatio = factor;
        setup(() -> motor.getConfigurator().apply(motorConfigs.ExternalFeedback), "Setup Position Conversion Factor");
    }

    /**
     * Set the conversion factor to convert from sensor velocity to mechanism velocity.
     * @param factor The conversion factor to multiply by
     */
    public void setupVelocityConversionFactor(double factor)
    {
        motorConfigs.ExternalFeedback.SensorToMechanismRatio = factor;
        setup(() -> motor.getConfigurator().apply(motorConfigs.ExternalFeedback), "Setup Velocity Conversion Factor");
    }

    /**
     * Checks if the slotId is valid (0-2)
     * @param slotId The PID slot (0-2)
     * @return True if the slotId is valid
     */
    private boolean isValidSlotId(int slotId)
    {
        return slotId >= 0 && slotId <= 2;
    }

    /**
     * Set the PID controls for the motor.
     * @param slotId The PID slot (0-2)
     * @param kP The Proportional constant
     * @param kI The Integral constant
     * @param kD The Derivative constant
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD)
    {
        setupPIDController(slotId, kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Set the PID controls for the motor.
     * @param slotId The PID slot (0-2)
     * @param kP The Proportional constant
     * @param kI The Integral constant
     * @param kD The Derivative constant
     * @param kS Static feedforward gain
     * @param kV Velocity feedforward gain
     * @param kA Acceleration feedforward gain
     * @param kG Gravity feedforward/feedback gain
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD, double kS, double kV, double kA, double kG)
    {
        if(isValidSlotId(slotId))
        {
            SlotConfigs slotConfigs = new SlotConfigs();

            slotConfigs.SlotNumber = slotId;
            slotConfigs.kP = kP;
            slotConfigs.kI = kI;
            slotConfigs.kD = kD;
            slotConfigs.kS = kS;
            slotConfigs.kV = kV;
            slotConfigs.kA = kA;
            slotConfigs.kG = kG;

            switch(slotId)
            {
                case 0:
                    motorConfigs.Slot0 = Slot0Configs.from(slotConfigs);
                    break;
                case 1:
                    motorConfigs.Slot1 = Slot1Configs.from(slotConfigs);
                    break;
                case 2:
                    motorConfigs.Slot2 = Slot2Configs.from(slotConfigs);
                    break;
            }
            setup(() -> motor.getConfigurator().apply(slotConfigs), "Setup PID Controller");
        }
    }

    /**
     * Returns the PID values stored in the slotId
     * @param slotId The PID slot (0-2)
     * @return An array containing the PID values
     */
    public double[] getPID(int slotId)
    {
        double[] pid = {0.0, 0.0, 0.0};
        if(isValidSlotId(slotId))
        {
            switch(slotId)
            {
                case 0:
                    pid[0] = motorConfigs.Slot0.kP;
                    pid[1] = motorConfigs.Slot0.kI;
                    pid[2] = motorConfigs.Slot0.kD;
                    break;
                case 1:
                    pid[0] = motorConfigs.Slot1.kP;
                    pid[1] = motorConfigs.Slot1.kI;
                    pid[2] = motorConfigs.Slot1.kD;
                    break;
                case 2:
                    pid[0] = motorConfigs.Slot2.kP;
                    pid[1] = motorConfigs.Slot2.kI;
                    pid[2] = motorConfigs.Slot2.kD;
                    break;
            }
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
        if(isInverted)
            setup(() -> motor.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed)), "Setup Follower");
        else
            setup(() -> motor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned)), "Setup Follower");
    }

    private void checkFault(StatusSignalFunction func)
    {
        if(func.doAction().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + func.doAction().getName());
            stickyFaultCount++;
        }
    }

    /**
     * Logs and then clears the sticky faults
     */
    public void logStickyFaults()
    {
        if(setupErrorCount > 0)
        {
            motorSetupPublisher.set(motorControllerName + " : " + setupErrorCount + " setup errors");
        }

        checkFault(() -> motor.getStickyFault_5V());
        checkFault(() -> motor.getStickyFault_BootDuringEnable());
        checkFault(() -> motor.getStickyFault_BridgeBrownout());
        checkFault(() -> motor.getStickyFault_BridgeShort());
        checkFault(() -> motor.getStickyFault_DeviceTemp());
        checkFault(() -> motor.getStickyFault_DriveDisabledHallSensor());
        checkFault(() -> motor.getStickyFault_ForwardHardLimit());
        checkFault(() -> motor.getStickyFault_ForwardSoftLimit());
        checkFault(() -> motor.getStickyFault_FusedSensorOutOfSync());
        checkFault(() -> motor.getStickyFault_HallSensorMissing());
        checkFault(() -> motor.getStickyFault_Hardware());
        checkFault(() -> motor.getStickyFault_MissingDifferentialFX());
        checkFault(() -> motor.getStickyFault_MissingHardLimitRemote());
        checkFault(() -> motor.getStickyFault_MissingSoftLimitRemote());
        checkFault(() -> motor.getStickyFault_MotorArrangementNotSelected());
        checkFault(() -> motor.getStickyFault_MotorTempSensorMissing());
        checkFault(() -> motor.getStickyFault_MotorTempSensorTooHot());
        checkFault(() -> motor.getStickyFault_OverSupplyV());
        checkFault(() -> motor.getStickyFault_ProcTemp());
        checkFault(() -> motor.getStickyFault_RemoteSensorDataInvalid());
        checkFault(() -> motor.getStickyFault_RemoteSensorPosOverflow());
        checkFault(() -> motor.getStickyFault_RemoteSensorReset());
        checkFault(() -> motor.getStickyFault_ReverseHardLimit());
        checkFault(() -> motor.getStickyFault_ReverseSoftLimit());
        checkFault(() -> motor.getStickyFault_StaticBrakeDisabled());
        checkFault(() -> motor.getStickyFault_StatorCurrLimit());
        checkFault(() -> motor.getStickyFault_SupplyCurrLimit());
        checkFault(() -> motor.getStickyFault_Undervoltage());
        checkFault(() -> motor.getStickyFault_UnlicensedFeatureInUse());
        checkFault(() -> motor.getStickyFault_UnstableSupplyV());
        checkFault(() -> motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed());

        // if(motor.getStickyFault_5V().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_5V);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_BootDuringEnable().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_BootDuringEnable);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_BridgeBrownout().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_BridgeBrownout);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_BridgeShort().getValue())
        // {
        //     motorFaultsPublisher.set(prefix  + SpnValue.StickyFault_TALONFX_BridgeShort);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_DeviceTemp().getValue())
        // {
        //     motorFaultsPublisher.set(prefix  + SpnValue.StickyFault_DeviceTemp);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_DriveDisabledHallSensor().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_DriveDisabledHallSensor);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_ForwardHardLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_ForwardHardLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_ForwardSoftLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_ForwardSoftLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_FusedSensorOutOfSync().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_FusedSensorOutOfSync);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_HallSensorMissing().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_HallSensorMissing);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_Hardware().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_Hardware);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MissingDifferentialFX().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MissingDifferentialFX);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MissingHardLimitRemote().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MissingRemHardLim);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MissingSoftLimitRemote().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MissingRemSoftLim);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MotorArrangementNotSelected().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MotorArrangementNotSelected);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MotorTempSensorMissing().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MotorTempSensorMissing);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_MotorTempSensorTooHot().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MotorTempSensorTooHot);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_OverSupplyV().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_OverSupplyV);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_ProcTemp().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_ProcTemp);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_RemoteSensorDataInvalid().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_MissingRemoteSensor);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_RemoteSensorPosOverflow().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_RemoteSensorPosOverflow);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_RemoteSensorReset().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_RemoteSensorReset);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_ReverseHardLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_ReverseHardLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_ReverseSoftLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_ReverseSoftLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_StaticBrakeDisabled().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_StaticBrakeDisabled);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_StatorCurrLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_StatorCurrLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_SupplyCurrLimit().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_SupplyCurrLimit);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_Undervoltage().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_Undervoltage);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_UnlicensedFeatureInUse().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_UnlicensedFeatureInUse);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_UnstableSupplyV().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_UnstableSupplyV);
        //     faultsCount++;
        // }
        // if(motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue())
        // {
        //     motorFaultsPublisher.set(prefix + SpnValue.StickyFault_TALONFX_UsingFusedCCWhileUnlicensed);
        //     faultsCount++;
        // }

        if(stickyFaultCount == 0)
        {
            motorFaultsPublisher.set(motorControllerName + " : " + "No Sticky Faults");
        }

        clearStickyFaults();
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
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
     * @param slotId The PID slot (0-2)
     */
    public void setControlPosition(double position, int slotId)
    {
        if(isValidSlotId(slotId))
        {
            positionVoltage.Slot = slotId;
            positionVoltage.Position = position;
            
            motor.setControl(positionVoltage);
        }
    }


    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
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
     * @param slotId The PID slot (0-2)
     */
    public void setControlVelocity(double velocity, int slotId)
    {
        if(isValidSlotId(slotId))
        {
            velocityVoltage.Slot = slotId;
            velocityVoltage.Velocity = velocity;

            motor.setControl(velocityVoltage);
        }
    }

    /**
     * Set the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position of the encoder
     */
    public void setPosition(double position)
    {
        motor.setPosition(position);
    }

    /**
     * Get the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @return The position of the encoder
     */
    public double getPosition()
    {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Get the velocity of the encoder.
     * Units are RPS by default, but can be changed using the conversion factor.
     * @return The velocity of the encoder
     */    
    public double getVelocity()
    {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the applied motor voltage (in volts).
     * @return The voltage
     */    
    public double getMotorVoltage()
    {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Get the supplied motor voltage (in volts).
     * @return The voltage
     */ 
    public double getMotorSupplyVoltage()
    {
        return motor.getSupplyVoltage().getValueAsDouble();
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
        return motorConfigs.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
    }

    @Override
    public void disable()
    {
        motor.disable();
    }
}
