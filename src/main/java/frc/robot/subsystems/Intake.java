package frc.robot.subsystems;

import static frc.robot.Constants.Intake.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.TalonFXLance;

/**
 * This is the Intake Subsystem
 * @author Niyati
 */
public class Intake extends SubsystemBase
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
    private final TalonFXLance RollersMotor = new TalonFXLance(ROLLER_MOTOR_LEADER, MOTOR_CAN_BUS, "Roller Motor");
    private final TalonFXLance RollersFollower = new TalonFXLance(ROLLER_MOTOR_FOLLOWER, MOTOR_CAN_BUS, "Follow Roller Motor");
    private final TalonFXLance PivotMotor = new TalonFXLance(PIVOT_MOTOR_LEADER, MOTOR_CAN_BUS, "Lead Pivot Motor");
    private final TalonFXLance PivotFollower = new TalonFXLance(PIVOT_MOTOR_FOLLOWER, MOTOR_CAN_BUS, "Follow Pivot Motor");

    

    private final double PivotkP = 0.3;
    private final double PivotkI = 0.0;
    private final double PivotkD = 0.0;
    private final double PivotkS = 0.19;
    private final double PivotkV = 0.13;

    private final double RollerkP = 0.2;
    private final double RollerkI = 0.0;
    private final double RollerkD = 0.0;
    private final double RollerkS = 0.19;
    private final double RollerkV = 0.13;

    private final double INTAKE_ROLLER_DIAMETER_FEET = 4237.0;
    private final double GEAR_RATIO = 1.0 / 1.0;
    private final double VELOCITY_CONVERSION_FACTOR = (Math.PI * INTAKE_ROLLER_DIAMETER_FEET) / GEAR_RATIO; // rev/s to ft/s using gear ratio // not checked

    private final double retractedPosition = 0.0;
    private final double intakingPosition = -11.0; // TEST FOR TRUE VALUE ON ROBOT
    // private final double theshold = 4.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Intake Subsystem. 
     */
    public Intake()
    {
        super("Intake");
        System.out.println("  Constructor Started:  " + fullClassName);
        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        // Factory Default all motors
        RollersMotor.setupFactoryDefaults();
        RollersFollower.setupFactoryDefaults();
        PivotMotor.setupFactoryDefaults();
        PivotFollower.setupFactoryDefaults();

        // Sets up motors to be inverted or not
        RollersMotor.setupInverted(true);
        PivotMotor.setupInverted(true);

        // Sets up Coast Mode
        RollersMotor.setupCoastMode();
        RollersFollower.setupCoastMode();
        PivotMotor.setupCoastMode();
        PivotFollower.setupCoastMode();

        // Sets up position
        RollersMotor.setPosition(0.0);
        RollersMotor.setPosition(0.0);
        PivotMotor.setPosition(0.0);
        PivotFollower.setPosition(0.0);

        // Set up Safety
        RollersMotor.setSafetyEnabled(false);
        RollersMotor.setSafetyEnabled(false);
        PivotMotor.setSafetyEnabled(false);
        PivotFollower.setSafetyEnabled(false);

        // Set up PID Controllers
        // RollersMotor.setupPIDController(0, RollerkP, RollerkI, RollerkD, RollerkS, RollerkV, 0);
        PivotMotor.setupPIDController(0, PivotkP, PivotkI, PivotkD, PivotkS, PivotkV, 0);
        // PivotFollower.setupPIDController(0, kP, kI, kD, kS, kV, 0);

        // Configure the follower last so configurables above are not overwritten

        RollersFollower.setupFollower(ROLLER_MOTOR_LEADER, true);
        PivotFollower.setupFollower(PIVOT_MOTOR_LEADER, true);

        // Configure Velocity Conversion Factor (rev/s to ft/s)
        // intakeRollersMotor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    }

    // private void set()
    // {
    //     intakePivotMotor.set(0.2);
    //     intakeRollersMotor.set(0.2);
    // }

    /**
     * This sets the voltage of the motors.
     * @param voltage
     */
    public void setVoltage(double voltage)
    {
        RollersMotor.setVoltage(voltage);
        PivotMotor.setVoltage(voltage);
    }

    /**
     * This sets the velocity of the motors.
     * @param velocity 
     */
    public void setVelocity(double velocity)
    {
        RollersMotor.setControlVelocity(velocity);
    }

    /** 
     * This method makes the intake pick up fuel
     */
    public void pickUpFuel()
    {
        RollersMotor.set(1.0);
        // setVelocity(20.0);
        // PivotMotor.setControlPosition(intakingPosition);
    }

    /** 
     * This method makes the intake eject fuel
     */
    public void ejectFuel()
    {
        setVelocity(-10.0);
        PivotMotor.setControlPosition(intakingPosition);
        // setVelocity(-0.20);
    }

    public void retractIntake()
    {
        PivotMotor.setControlPosition(retractedPosition);
        setVelocity(10.0);
    }

    public BooleanSupplier isAtPosition(double desiredPosition)
    {
        return () -> (Math.abs(PivotMotor.getPosition() - desiredPosition) <= 1.0);
    }

    /**
     * This method stops the intake motors
     */
    public void stop()
    {
        // intakePivotMotor.set(0.0);
        // intakeRollersMotor.set(0.0);
        setVelocity(0.0);
    }

    public Command setVelocityCommand(DoubleSupplier speed)
    {
        return run( () -> setVelocity(speed.getAsDouble()));
    }

    public Command pickupFuelCommand()
    {
        return run(() -> pickUpFuel()).withName("Pickup Fuel");
    }

    public Command ejectFuelCommand() 
    {
        return run(() -> ejectFuel()).withName("Eject Fuel");
    }

    public Command retractIntakeCommand()
    {
        return run(() -> retractIntake()).withName("Retract Intake");
    }

    public Command onCommand()
    {
        return run( () -> setVelocity(0.25) );
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return runOnce(() -> stop()).withName("Stop Intake");
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // if (RollersMotor.getCurrentAmps() > 20.0)
            System.out.println("RollerMotor amps: " + RollersMotor.getCurrentAmps() + " FollowerRoller Amps: " + RollersFollower.getCurrentAmps());

        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
    }

    @Override
    public String toString()
    {
        return "Current Intake Velocity: " + PivotMotor.getVelocity();
    }
}