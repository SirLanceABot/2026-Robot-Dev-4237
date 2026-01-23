package frc.robot.subsystems;


import static frc.robot.Constants.Flywheel.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Flywheel extends SubsystemBase
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
    private final TalonFXLance leadMotor = new TalonFXLance(LEADMOTOR, MOTOR_CAN_BUS, "Flywheel Lead Motor");
    private final TalonFXLance followMotor = new TalonFXLance(FOLLOWMOTOR, MOTOR_CAN_BUS, "Flywheel Follow Motor");

    private final TalonFX motor = new TalonFX(1, MOTOR_CAN_BUS);

    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    
    // PID constants
    private final double kP = 0.3;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.19;
    private final double kV = 0.13;
    private final double kA = 0.01;


    private final double velocityConversionFactor = 1.0; // figure out units (currently default rev/sec)


    InterpolatingDoubleTreeMap scoreMap = new InterpolatingDoubleTreeMap();
    // InterpolatingDoubleTreeMap passLeftMap = new InterpolatingDoubleTreeMap();
    // InterpolatingDoubleTreeMap passRightMap = new InterpolatingDoubleTreeMap();


    // private final PIDController flywheelPID = new PIDController(0, 0, 0);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Shooter. 
     */
    public Flywheel()
    {
        super("Flywheel");
        
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();
        configScoreMap();
        // configLeftPassMap();
        // configRightPassMap();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        leadMotor.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();

        followMotor.setupFollower(LEADMOTOR, false);

        leadMotor.setSafetyEnabled(true);
        followMotor.setSafetyEnabled(true);

        leadMotor.setupCoastMode();
        followMotor.setupCoastMode();

        leadMotor.setupPIDController(0, kP, kI, kD, kS, kV, kA);
        
        leadMotor.setupVelocityConversionFactor(velocityConversionFactor);

        // var talonFXConfigs = new TalonFXConfiguration();

        // var slot0configs = talonFXConfigs.Slot0;
        // slot0configs.kP = kP;
        // slot0configs.kI = kI;
        // slot0configs.kD = kD;
        // slot0configs.kS = kS;
        // slot0configs.kV = kV;
        // slot0configs.kA = kA;

        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        // motionMagicConfigs.MotionMagicExpo_kV = kV;
        // motionMagicConfigs.MotionMagicExpo_kA = 0.1;

        // leadMotor.getConfigurator().apply(talonFXConfigs);

        // motor.getConfigurator().apply(talonFXConfigs);
    }

    private void configScoreMap()
    {
        // first value is distance (ft) from the hub (in alliance zone), second is flywheel velo
        
        // calculated distances using phsyics model
        scoreMap.put(0.0, 4237.0);
        scoreMap.put(1.0, 4237.0);
        scoreMap.put(2.0, 4237.0);
        scoreMap.put(3.0, 4237.0);
        scoreMap.put(4.0, 4237.0);
        scoreMap.put(5.0, 4237.0);
        scoreMap.put(6.0, 4237.0);
        scoreMap.put(7.0, 4237.0);
        scoreMap.put(8.0, 4237.0);
        scoreMap.put(9.0, 4237.0);
        scoreMap.put(10.0, 4237.0);
        scoreMap.put(11.0, 4237.0);
        scoreMap.put(12.0, 4237.0);
        scoreMap.put(13.0, 4237.0);
        scoreMap.put(14.0, 4237.0);
        scoreMap.put(15.0, 4237.0);
        scoreMap.put(16.0, 4237.0);
        scoreMap.put(17.0, 4237.0);
        scoreMap.put(18.0, 4237.0);
        scoreMap.put(19.0, 4237.0);
        scoreMap.put(20.0, 4237.0);
    }

    // idk if we need these maps or not
    // private void configLeftPassMap()
    // {}

    // private void configRightPassMap()
    // {}

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        leadMotor.set(speed);
        // followMotor.set(speed);
    }
    
    private void setControlVelocity(double speed)
    {
        leadMotor.setControlVelocity(speed);
        // followMotor.setControlVelocity(speed);
    }

    private void burpFuel()
    {
        leadMotor.setControlVelocity(-10.0);
    }

    private void stop()
    {
        set(0.0);
    }

    private void shoot(double speed)
    {
        setControlVelocity(speed);
    }

    public void setVoltage(double voltage)
    {
        leadMotor.setVoltage(voltage);
    }

    public void motionMagic(double position)
    {
        motor.setControl(m_request.withPosition(position));
    }

    public double getVelocity()
    {
        return leadMotor.getVelocity();
    }

    // private double calcScoreVelocityFromDistance()
    // {}

    // more than one passing location?
    // private double calcPassVelocityFromDistance()
    // {}

    public Command onCommand()
    {
        return run( () -> set(0.1) );
    }

    public Command shootCommand(DoubleSupplier speed)
    {
        return run( () -> shoot(speed.getAsDouble()));
    }

    public Command burpFuelCommand()
    {
        return run( () -> burpFuel());
    }

    public BooleanSupplier isAtSetSpeed(double targetSpeed, double speedTolerance)
    {
        double currentSpeed = leadMotor.getVelocity();
        
        return () ->
        {
        if((currentSpeed + speedTolerance > targetSpeed) && (currentSpeed - speedTolerance < targetSpeed))
        {
            return true;
        }
        else
        {
            return false;
        }
        };
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return runOnce( () -> stop());
    }

    public Command runMotorUsingVoltageCommand(double voltage)
    {
        return run( () -> setVoltage(voltage));
    }

    public Command useMotionMagicCommand(double position)
    {
        return run( () -> motionMagic(position));
    }

    // public Command runMotorUsingFFCommand(double speed)
    // {
    //     return run( () -> runMotorUsingFF(speed));
    // }


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
        return "Flywheel Velocity = " + getVelocity();
    }
}
