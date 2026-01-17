package frc.robot.subsystems;

import static frc.robot.Constants.CANbus.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
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
        kL1(8), kL2(4237), kL3(4237), kSTART(0.0);

        public final double value;
        private climbPosition(double value)
        {
            this.value = value;
        }
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance leadMotor = new TalonFXLance(Constants.Climb.MOTOR1, "ROBORIO", "Motor 1");
    private final TalonFXLance followMotor = new TalonFXLance(Constants.Climb.MOTOR2, "ROBORIO", "Motor 2");
    
    private final double tolerance = 2048/4;

    private static final double kPUP = 0.5;
    private static final double kPDOWN = 0.1;
    private static final double kI = 0;
    private static final double kD = 0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Climb()
    {
        super("Example Subsystem");
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

        leadMotor.setupForwardSoftLimit(0, false);
        followMotor.setupForwardSoftLimit(0, false);

        leadMotor.setupReverseSoftLimit(0, false);
        followMotor.setupReverseSoftLimit(0, false);

        followMotor.setupFollower(Constants.Climb.MOTOR1, true);


        leadMotor.setupPIDController(0, kPUP, kI, kD); 
        leadMotor.setupPIDController(1, kPDOWN, kI, kD); 

    }

    private double getPosition()
    {
        return leadMotor.getPosition();
    }

     public BooleanSupplier isAtPosition(climbPosition position)
    {
        return () -> Math.abs(leadMotor.getPosition() - position.value) < tolerance;
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

    public Command stopCommand()
    {
        return run( () -> stop());
    }

    public Command ascendL1Command()
    {
        return run( () -> moveToPosition(climbPosition.kL1)).until(isAtPosition(climbPosition.kL1));
    }
    
    public Command descendL1Command()
    {
        return run( () -> moveToPosition(climbPosition.kSTART)).until(isAtPosition(climbPosition.kL1));
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
