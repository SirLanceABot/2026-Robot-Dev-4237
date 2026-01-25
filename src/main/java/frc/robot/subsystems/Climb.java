package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.TalonFXLance;


public class Climb extends SubsystemBase
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
    private enum climbPosition
    {
        kL1(32), kL2(4237), kL3(4237), kSTART(0.0);

        public final double value;
        private climbPosition(double value)
        {
            this.value = value;
        }
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance leadMotor = new TalonFXLance(LEADMOTOR, MOTOR_CAN_BUS, "Lead Climb Motor ");
    private final TalonFXLance followMotor = new TalonFXLance(FOLLOWMOTOR, MOTOR_CAN_BUS, "Follower Climb Motor");
    
    private final double tolerance = 0.2;

    private static final double kPUP = 9.9;
    private static final double kPDOWN = 9.9;
    private static final double kI = 0;
    private static final double kD = 0.0;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a Climb. 
     */
    public Climb()
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
        leadMotor.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();

        leadMotor.setupBrakeMode();
        followMotor.setupBrakeMode();

        leadMotor.setPosition(0.0);
        followMotor.setPosition(0.0);

        // leadMotor.setupForwardSoftLimit(16, true);
        // followMotor.setupForwardSoftLimit(0, false);

        // leadMotor.setupReverseSoftLimit(0.0, true);
        // followMotor.setupReverseSoftLimit(0, false);

        leadMotor.setupForwardHardLimitSwitch(true, true, 0);
        leadMotor.setupReverseHardLimitSwitch(true, true, 1);

        followMotor.setupFollower(LEADMOTOR, true);

        leadMotor.setupPIDController(0, kPUP, kI, kD); 
        leadMotor.setupPIDController(1, kPDOWN, kI, kD); 

    }

    public boolean getForwardHardLimit()
    {
        return leadMotor.getForwardHardLimit();
    }

    public double getPosition()
    {
        return leadMotor.getPosition();
    }


    public BooleanSupplier isAtPosition(climbPosition position)
    {
        return () -> Math.abs(leadMotor.getPosition() - position.value) < tolerance;
        // return () -> false;
    }

    private void stop()
    {
        leadMotor.set(0.0);
    }

    private void moveToPosition(climbPosition targetPosition)
    {
        if(targetPosition.value < getPosition())
        {
            leadMotor.setControlPosition(targetPosition.value, 0);
        }
        else if(targetPosition.value > getPosition())
        {
            leadMotor.setControlPosition(targetPosition.value, 1);

        }
    }

    /**
     * 
     * Returns a command to stop climb
     * @return stop climb method
     */
    public Command stopCommand()
    {
        return run( () -> stop());
    }

    /**
     * @author Robbie J
     * @return move to position L1, until at position or hard limit hit; then stops 
     */
    public Command ascendL1Command()
    {
        return run( () -> moveToPosition(climbPosition.kL1)).until(() -> (isAtPosition(climbPosition.kL1).getAsBoolean() || (leadMotor.getForwardHardLimit())))
                .andThen(stopCommand());
    }

    /**
     * @author Robbie J
     * @return move to position start, until at position or hard limit hit; then stops 
     */
    public Command descendL1Command()
    {
        return run( () -> moveToPosition(climbPosition.kSTART)).until(() -> (isAtPosition(climbPosition.kSTART).getAsBoolean() || (leadMotor.getReverseHardLimit())))
        .andThen(stopCommand());
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
        return "";
    }
}
