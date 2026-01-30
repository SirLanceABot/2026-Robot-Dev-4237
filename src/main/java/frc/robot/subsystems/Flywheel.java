package frc.robot.subsystems;


import static frc.robot.Constants.Flywheel.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.TakeBackHalfController;
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

    // private final TalonFX motor = new TalonFX(1, MOTOR_CAN_BUS);
    
    private final double defaultGain = 1.e-5;
    private final TakeBackHalfController TBHController = new TakeBackHalfController(defaultGain, 0.05);

    // PID constants
    private final double kP = 0.2;
    private final double kI = 0.0;
    private final double kD = 0.00;
    private final double kS = 0.19;
    private final double kV = 0.13;
    private final double kA = 0.00;

    private final double VELOCITYCONVERSIONFACTOR = 1.0; // figure out units (currently default rev/sec)

    // Motion Magic Constants
    private final double MOTIONMAGICCRUISEVELOCITY = 100.0; // target cruise velocity
    private final double MOTIONMAGICACCELERATION = 30.0; // target acceleration
    private final double MOTIONMAGICJERK = 100.0; // target jerk


    InterpolatingDoubleTreeMap distToVeloMap = new InterpolatingDoubleTreeMap();

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
        configShotMap();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        leadMotor.setupFactoryDefaults();
        followMotor.setupFactoryDefaults();

        leadMotor.setSafetyEnabled(true);
        followMotor.setSafetyEnabled(true);

        leadMotor.setupCoastMode();
        followMotor.setupCoastMode();

        leadMotor.setupPIDController(0, kP, kI, kD, kS, kV, kA);

        // leadMotor.setupTorqueControl();
        // followMotor.setupTorqueControl();
        
        leadMotor.setupVelocityConversionFactor(VELOCITYCONVERSIONFACTOR);

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

        // leadMotor.setupMotionMagicControl(MOTIONMAGICCRUISEVELOCITY, MOTIONMAGICACCELERATION, MOTIONMAGICJERK);

        followMotor.setupFollower(LEADMOTOR, false);
    }

    private void configShotMap()
    {
        // first value is distance (ft) from the hub (in alliance zone), second is flywheel velo (ft/s)
        // TODO test values once we have robot
        distToVeloMap.put(3.0, 4237.0);
        distToVeloMap.put(4.0, 4237.0);
        distToVeloMap.put(5.0, 4237.0);
        distToVeloMap.put(6.0, 4237.0);
        distToVeloMap.put(7.0, 4237.0);
        distToVeloMap.put(8.0, 4237.0);
        distToVeloMap.put(9.0, 4237.0);
        distToVeloMap.put(10.0, 4237.0);
        distToVeloMap.put(11.0, 4237.0);
        distToVeloMap.put(12.0, 4237.0);
        distToVeloMap.put(13.0, 4237.0);
        distToVeloMap.put(14.0, 4237.0);
        distToVeloMap.put(15.0, 4237.0);
        distToVeloMap.put(16.0, 4237.0);
        distToVeloMap.put(17.0, 4237.0);
        distToVeloMap.put(18.0, 4237.0);
        distToVeloMap.put(19.0, 4237.0);
        distToVeloMap.put(20.0, 4237.0);
    }

    public double getShotPower(double dist)
    {
        dist = Math.max(4.0, Math.min(20.0, dist));
        return distToVeloMap.get(dist);
    }

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

    public void setVoltage(double voltage)
    {
        leadMotor.setVoltage(voltage);
    }
    
    // uses the Take-Back-Half controller to control velocity
    public void useTBH(double speed)
    {
        TBHController.setSetpoint(speed, speed);
        leadMotor.set(TBHController.calculate(getVelocity()));
    } 

    public double getVelocity()
    {
        return leadMotor.getVelocity();
    }

    // public void displayStatorCurrent()
    // {
    //     System.out.println("Lead Motor Stator Current = " + leadMotor.motor.getStatorCurrent());
    //     System.out.println("Follow Motor Stator Current = " + followMotor.motor.getStatorCurrent());
    // }

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

    public Command onCommand()
    {
        return run( () -> set(0.1) );
    }
    

    /**
     * @param speed to spin flywheel at
     * @return Command to set the control velo of the flywheel, should be using Torque FOC
     */
    public Command setControlVelocityCommand(DoubleSupplier speed)
    {
        return run( () -> setControlVelocity(speed.getAsDouble()));
    }

    public Command burpFuelCommand()
    {
        return run( () -> burpFuel());
    }

    // Use a method reference instead of this method
    public Command stopCommand()
    {
        return run(this::stop);
    }

    public Command runMtorUsingVoltageCommand(double voltage)
    {
        return run( () -> setVoltage(voltage));
    }

    public Command useTBHCommand(double setpoint)
    {
        return run(() -> useTBH(setpoint));
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {

        // System.out.println("*****Velocity Difference = +/- " + (leadMotor.getVelocity()-followMotor.getVelocity()));

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
