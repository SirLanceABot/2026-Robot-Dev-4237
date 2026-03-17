package frc.robot.subsystems;


import static frc.robot.Constants.Flywheel.*;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final TalonFXLance masterMotor = new TalonFXLance(MASTERMOTOR, MOTOR_CAN_BUS, "Flywheel Master Motor"); //X60
    private final TalonFXLance slave1Motor = new TalonFXLance(SLAVE1MOTOR, MOTOR_CAN_BUS, "Flywheel Slave1 Motor"); //X60
    private final TalonFXLance slave2Motor = new TalonFXLance(SLAVE2MOTOR, MOTOR_CAN_BUS, "Flywheel Slave2 Motor"); //X60
    
    private final double defaultGain = 1.e-5;
    private final TakeBackHalfController TBHController = new TakeBackHalfController(defaultGain, 0.05);
    

    // PID constants
    private final double kP = 3.6; // 5
    private final double kI = 0.0; // 0.4
    private final double kD = 0.05; // 0.0
    private final double kS = 0.02; // 0.016
    private final double kV = 0.0; //0.0105 * 12;
    private final double kA = 0.00;

    private final double FLYWHEEL_DIAMETER_FEET  = (4.0 / 12.0); // 4.25 in
    private final double GEAR_RATIO = 1.0; // 16.0 / 24.0
    private final double VELOCITY_CONVERSION_FACTOR = (Math.PI * FLYWHEEL_DIAMETER_FEET) / GEAR_RATIO; // rev/s to ft/s using gear ratio // checked
    
    // Motion Magic Constants
    private final double MOTIONMAGICCRUISEVELOCITY = 70.0; // target cruise velocity
    private final double MOTIONMAGICACCELERATION = 25.0; // target acceleration
    private final double MOTIONMAGICJERK = 3000.0; // target jerk

    // private final DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, 20, 50).withJerk(4000);

    InterpolatingDoubleTreeMap distToVeloMap = new InterpolatingDoubleTreeMap();
    // InterpolatingDoubleTreeMap distToPassVeloMap = new InterpolatingDoubleTreeMap();

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

        SmartDashboard.putNumber("Flywheek kV", 0);
        configMotors();
        configShotMap();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        masterMotor.setupFactoryDefaults();
        slave1Motor.setupFactoryDefaults();
        slave2Motor.setupFactoryDefaults();

        masterMotor.setSafetyEnabled(false);
        slave1Motor.setSafetyEnabled(false);
        slave2Motor.setSafetyEnabled(false);

        masterMotor.setupCoastMode();
        slave1Motor.setupCoastMode();
        slave2Motor.setupCoastMode();

        // leadMotor.setupPIDController(0, kP, kI, kD, kV);
        masterMotor.setupPIDController(0, kP, kI, kD);
        // followMotor.setupPIDController(1, kP, kI, kD, kV);
        // leadMotor.setupOpenLoopRampRate(5.0);
        // followMotor.setupOpenLoopRampRate(5.0);

        masterMotor.setupTorqueControl();
        slave1Motor.setupTorqueControl();
        slave2Motor.setupTorqueControl();
        
        masterMotor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        slave1Motor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        slave2Motor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        slave1Motor.setupFollower(MASTERMOTOR, false);
        slave2Motor.setupFollower(MASTERMOTOR, true);

        // TODO see if makes a difference
        masterMotor.setVoltageUpdateFrequency(200);
        masterMotor.setTorqueCurrentUpdateFrequency(1000);
    }
 
    private void configShotMap()
    {
        // first value is distance (ft) from the hub (in alliance zone), second is flywheel velo (currently ft/s)
        // TODO redo fr
        distToVeloMap.put(6.0, 41.5); // 41.5
        distToVeloMap.put(7.0, 41.5); // 42
        distToVeloMap.put(8.0, 42.5); // 42.5 // added 0.5
        distToVeloMap.put(9.0, 44.0); // 45.5
        distToVeloMap.put(10.0, 45.5); // 47.25
        distToVeloMap.put(11.0, 47.25); // 49.0
        distToVeloMap.put(12.0, 49.0); // 51.75
        distToVeloMap.put(13.0, 52.0); // 53.75
        distToVeloMap.put(14.0, 54.0); // 59.0
        distToVeloMap.put(15.0, 54.75); // 60 // last 0.5
        distToVeloMap.put(16.0, 55.5); // 60
        distToVeloMap.put(17.0, 57.5); // 60
        distToVeloMap.put(18.0, 59.5);
        distToVeloMap.put(19.0, 62.0);
        distToVeloMap.put(20.0, 63.5);
        distToVeloMap.put(21.0, 68.5);
        distToVeloMap.put(22.0, 71.5);
        distToVeloMap.put(23.0, 75.5);
        distToVeloMap.put(24.0, 80.0);
        distToVeloMap.put(25.0, 84.0);
    }

    // private void configPassMap()
    // {
    //     distToPassVeloMap.put(20.0, 4237.0);
    //     distToPassVeloMap.put(30.0, 4237.0);
    // }



    /**
     * @param dist from alliance hub
     * @return corrected distance from hub
     */
    public double getShotPower(double dist)
    {
        dist = Math.max(6.0, Math.min(25.0, dist));
        return distToVeloMap.get(dist);
    }

    // public double getPassPower(double dist)
    // {
    //     dist = Math.max(10.0, Math.min(50.0, dist));
    //     return distToPassVeloMap.get(dist);
    // }

    public double getDutyCycle()
    {
        return masterMotor.get();
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    private void set(double speed)
    {
        masterMotor.set(speed);
        // followMotor.set(speed);
    }
    
    /**
     * @param speed velocity to set flywheel to
     */
    private void setControlVelocity(double speed)
    {
        masterMotor.setControlVelocity(speed);
        // followMotor.setControlVelocity(speed);
    }

    /**
     * burps fuel at 10.0 rps
     */
    private void burpFuel()
    {
        masterMotor.setControlVelocity(30.0);
    }

    /**
     * method to stop motors
     */
    private void stop()
    {
        set(0.0);
    }

    /**
     * @param voltage to run motor at 
     */
    public void setVoltage(double voltage)
    {
        masterMotor.setVoltage(voltage);
    }
    
    /**
     * uses TBH to control velocity
     * @param speed velocity to spin flywheel at
     */
    public void useTBH(double speed)
    {
        TBHController.setSetpoint(speed, speed); // gain should not be speed (bad)
        masterMotor.set(TBHController.calculate(getVelocity()));
    } 

    /**
     * @return velocity of lead motor
     */
    public double getVelocity()
    {
        return masterMotor.getVelocity();
    }

    // public void displayStatorCurrent()
    // {
    //     System.out.println("Lead Motor Stator Current = " + leadMotor.motor.getStatorCurrent());
    //     System.out.println("Follow Motor Stator Current = " + followMotor.motor.getStatorCurrent());
    // }

    /**
     * @param targetSpeed to spin flywheel at
     * @param speedTolerance speed tolerance
     * @return boolean for if flywhele is at speed
     */
    public BooleanSupplier isAtSetSpeed(double targetSpeed, double speedTolerance)
    {
        double currentSpeed = masterMotor.getVelocity();
        
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
        return run( () -> set(0.5) );
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
        return runOnce(this::stop);
    }

    public Command setCommand()
    {
        return run(() -> set(0.5));
    }

    public Command runMotorUsingVoltageCommand(double voltage)
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
        // double testKV = SmartDashboard.getNumber("Flywheek kV", 0);
        // leadMotor.setupPIDController(0, kP, kI, kD, kS, testKV, kA);
        // System.out.println("*****lead motor velocity (fps) = " + leadMotor.getVelocity());
        // System.out.print("-----follow motor velocity (fps) = " + followMotor.getVelocity());
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
