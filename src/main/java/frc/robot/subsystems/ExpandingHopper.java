package frc.robot.subsystems;

import static frc.robot.Constants.ExpandingHopper.MOTOR;
import static frc.robot.Constants.ExpandingHopper.MOTOR_CAN_BUS;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.TalonFXLance;


public class ExpandingHopper extends SubsystemBase
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private final double tolerance = 0.0;

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here
    public enum hopperPosition
    {
        kRETRACTED(0.0), kEXTENDED(31.0);

        public final double value;
        private hopperPosition(double value)
        {
            this.value = value;
        }
    }
    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private final TalonFXLance motor = new TalonFXLance(MOTOR, MOTOR_CAN_BUS, "Hopper Motor ");

    private static final double kPUP = .45;
    private static final double kPDOWN = .45;
    private static final double kI = 0.0;
    private static final double kD = 0.0;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates an Expanding Hopper. 
     */
    public ExpandingHopper()
    {
        super("Climb Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();

        motor.setupBrakeMode();

        motor.setPosition(0.0);

        motor.setSafetyEnabled(false);

        motor.setupInverted(true);

        motor.setupForwardSoftLimit(32.0, true);

        motor.setupReverseSoftLimit(-1.0, true);

        // motor.setupForwardHardLimitSwitch(true, true, 1);

        motor.setupReverseHardLimitSwitch(true, true, 2);

        // motor.setupFollower(LEADMOTOR, true);

        motor.setupPIDController(0, kPUP, kI, kD); 
        motor.setupPIDController(1, kPDOWN, kI, kD); 

    }

    public boolean getForwardHardLimit()
    {
        return motor.getForwardHardLimit();
    }

    public double getPosition()
    {
        return motor.getPosition();
    }

    public void runHopperMotor()
    {
        motor.set(0.3);
    }

    // figure out if up is positive velo
    public void manualExtendHopper()
    {
        motor.set(0.25);
    }

    public void manualRetractHopper()
    {
        motor.set(-0.25);
    }

    /**
     * @param ExpandingHopper position
     * @return Boolean suplier of is at a postion within tolerance
     */
    public BooleanSupplier isHopperAtPosition(hopperPosition position)
    {
        return () -> Math.abs(motor.getPosition() - position.value) < tolerance;
    }

    private void stopMotor()
    {
        motor.set(0.0);
    }

    /**
     * Moves to target position using a PID, slot 0 for up slot 1 for down
     * @param targetPosition
     */
    private void moveToPosition(hopperPosition targetPosition)
    {
        if(targetPosition.value < getPosition())
        {
            motor.setControlPosition(targetPosition.value, 0);
        }
        else if(targetPosition.value > getPosition())
        {
            motor.setControlPosition(targetPosition.value, 1);

        }
    }

    public void resetPosition()
    {
        motor.setPosition(0.0);
    }

    /**
     * 
     * Returns a command to stop climb
     * @return stop climb method
     */
    public Command stopMotorCommand()
    {
        return runOnce( () -> stopMotor());
    }

    public Command extendHopperCommand()
    {
        return run( () -> moveToPosition(hopperPosition.kEXTENDED))
            .until(isHopperAtPosition(hopperPosition.kEXTENDED));
    }
    
    public Command retractHopperCommand()
    {
        return run( () -> moveToPosition(hopperPosition.kRETRACTED))
            .until(isHopperAtPosition(hopperPosition.kRETRACTED));
    }

    public Command manualExtendHopperCommand()
    {
        return run( () -> manualExtendHopper());
    }

    public Command manualRetractHopperCommand()
    {
        return run( () -> manualRetractHopper());
    }

    // *** OVERRIDEN METHODS ***!
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        // System.out.println("**hopper posiition = " + motor.getPosition());
        // System.out.println("ispressed: " + motor.getReverseHardLimit());
    }

    @Override
    public String toString()
    {
        return "";
    }
}
