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
        kL1(4237), kL2(4237), kL3(4237), kSTART(4237);

        public final double value;
        private climbPosition(double value)
        {
            this.value = value;
        }
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance motor1 = new TalonFXLance(Constants.Climb.MOTOR1, Constants.Climb.MOTOR_CAN_BUS, "Motor 1");
    private final TalonFXLance motor2 = new TalonFXLance(Constants.Climb.MOTOR2, Constants.Climb.MOTOR_CAN_BUS, "Motor 2");
    
    private final double tolerance = 4237;

    private static final double kPUP = 4237;
    private static final double kPDOWN = 4237;
    private static final double kI = 4237;
    private static final double kD = 4237;

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
        motor1.setupFactoryDefaults();
        motor2.setupFactoryDefaults();

        motor1.setupBrakeMode();
        motor2.setupBrakeMode();

        motor1.setPosition(0.0);
        motor2.setPosition(0.0);

        motor1.setupForwardSoftLimit(0, false);
        motor2.setupForwardSoftLimit(0, false);

        motor1.setupReverseSoftLimit(0, false);
        motor2.setupReverseSoftLimit(0, false);

        motor1.setupPIDController(0, kPUP, kI, kD); 
        motor1.setupPIDController(1, kPDOWN, kI, kD); 

    }

    private double getPosition()
    {
        return motor1.getPosition();
    }

     public BooleanSupplier isAtPosition(climbPosition position)
    {
        return () -> Math.abs(motor1.getPosition() - position.value) < tolerance;
    }

    private void moveToPosition(climbPosition targetPosition)
    {
        if(targetPosition.value < getPosition())
        {
            motor1.setControlPosition(targetPosition.value, 0);
        }
        else if(targetPosition.value > getPosition())
        {
            motor1.setControlPosition(targetPosition.value, 1);

        }
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
