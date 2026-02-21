package frc.robot.subsystems;

import static frc.robot.Constants.Climb.LEADMOTOR;
import static frc.robot.Constants.Climb.MOTOR_CAN_BUS;
import static frc.robot.Constants.Climb.SENSOR_PORT;
import static frc.robot.Constants.Climb.SERVOMOTOR;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
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
    public enum climbPosition
    {
        kEXTENDL1(100.0), kRETRACTL1(30.0), kL2(4237), kL3(4237), kSTART(0.0); // 32.0, 10.0

        public final double value;
        private climbPosition(double value)
        {
            this.value = value;
        }
    }

    public enum servoPosition
    {
        kSTARt(0), kRETRACTED((int) ((MIN_SERVO_LENGTH / 50.0) * 1000) + 1000), kEXTENDED((int) ((MAX_SERVO_LENGTH / 50.0) * 1000) + 1000);

        public final int value;
        private servoPosition(int value)
        {
            this.value = value;
        }
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private static final double MAX_SERVO_LENGTH = 38.634;      // needs to be tested on actual bot
    private static final double MIN_SERVO_LENGTH = 10.449;      // needs to be tested on actual bot

    private final TalonFXLance leadMotor = new TalonFXLance(LEADMOTOR, MOTOR_CAN_BUS, "Lead Climb Motor ");
    // private final TalonFXLance followMotor = new TalonFXLance(FOLLOWMOTOR, MOTOR_CAN_BUS, "Follower Climb Motor");
    
    private final Servo servo = new Servo(SERVOMOTOR);

    private final DigitalInput climbSensor = new DigitalInput(SENSOR_PORT);

    

    private final double tolerance = 0.2;
    private final double servoTolerance = 0.05;

    private static final double kPUP = 5.0; //9.9;
    private static final double kPDOWN = 5.0; //9.9;
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
        // followMotor.setupFactoryDefaults();

        leadMotor.setupBrakeMode();
        // followMotor.setupBrakeMode();

        leadMotor.setPosition(0.0);
        // followMotor.setPosition(0.0);

        leadMotor.setSafetyEnabled(false);

        // leadMotor.setupForwardSoftLimit(32.0, true);
        // followMotor.setupForwardSoftLimit(0, false);

        // leadMotor.setupReverseSoftLimit(0.0, true);
        // followMotor.setupReverseSoftLimit(0.0, false);

        // leadMotor.setupForwardHardLimitSwitch(true, true, 0);
        // leadMotor.setupReverseHardLimitSwitch(true, true, 1);

        // followMotor.setupFollower(LEADMOTOR, true);

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

    public void runClimb()
    {
        leadMotor.set(0.3);
    }


    public BooleanSupplier isClimbMotorAtPosition(climbPosition position)
    {
        return () -> Math.abs(leadMotor.getPosition() - position.value) < tolerance;
        // return () -> false;
    }

    public BooleanSupplier isServoAtPosition(double position)
    {
        return () -> Math.abs(leadMotor.getPosition() - position) < servoTolerance;
    }

    private void stopMotor()
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

    public void resetPosition()
    {
        leadMotor.setPosition(0.0);
    }

    /**
     * @param position to set the servo to (0.0 - 1.0)
     */
    public void setServoPosition(double position)
    {
        servo.setPosition(position);
    }

    public double getServoPosition()
    {
        return servo.getPosition();
    }

    /**
     * @param pulse PWM pulse to set the servo to (1000-2000)
     */
    public void setServoPWM(servoPosition pulse)
    {
        servo.setPulseTimeMicroseconds(pulse.value);
    }

    /**
     * @return staus of the climb sensor
     */
    public boolean getClimbSensor()
    {
        return climbSensor.get();
    }

    /**
     * @author Robbie F
     * @param position that the climb must reach before it starts reading the center
     * @param isForward is the motor moving forward or backward
     * @return status of the climb sensor after specified position has elapsed
     */
    public boolean getClimbSensorAfterDistance(double position, boolean isForward)
    {
        if(isForward)
        {
            if(getPosition() > position)
                return climbSensor.get();
        
            return false;
        }
        else
        {
            if(getPosition() < position)
                return climbSensor.get();
            
            return false;
        }
        
    }


    // *****climb motor*****

    /**
     * 
     * Returns a command to stop climb
     * @return stop climb method
     */
    public Command stopMotorCommand()
    {
        return runOnce( () -> stopMotor());
    }

    /**
     * @author Robbie
     * climb motor stage 1
     * @return the command
     */
    public Command extendToL1FromStartCommand()
    {
        return run( () -> moveToPosition(climbPosition.kEXTENDL1)).until(()-> (isClimbMotorAtPosition(climbPosition.kEXTENDL1).getAsBoolean() || isDetectedAfterDistanceSupplier(climbPosition.kSTART.value + 1.0, true).getAsBoolean()))
                .andThen(stopMotorCommand());
    }

    /**
     * @author Robbie
     * climb motor stage 2
     * @return the command
     */
    public Command extendToL1FromRetractedCommand()
    {
        return run( () -> moveToPosition(climbPosition.kEXTENDL1)).until(()-> (isClimbMotorAtPosition(climbPosition.kEXTENDL1).getAsBoolean() || isDetectedAfterDistanceSupplier(climbPosition.kRETRACTL1.value + 1.0, true).getAsBoolean()))
                .andThen(stopMotorCommand());
    }

    /**
     * @author Robbie
     * climb motor stage 3
     * @return the command
     */
    public Command retractFromExtendL1Command()
    {
        return run( () -> moveToPosition(climbPosition.kRETRACTL1)).until(() -> (isClimbMotorAtPosition(climbPosition.kRETRACTL1).getAsBoolean() || isDetectedAfterDistanceSupplier(climbPosition.kEXTENDL1.value - 1.0, false).getAsBoolean()))
            .andThen(stopMotorCommand());
    }

    /**
     * @author Robbie
     * climb motor stage 4
     * @return the command
     */
    public Command resetToStartFromExtendedCommand()
    {
        return run( () -> moveToPosition(climbPosition.kSTART)).until(() -> (isClimbMotorAtPosition(climbPosition.kSTART).getAsBoolean() || isDetectedAfterDistanceSupplier(climbPosition.kEXTENDL1.value - 1.0, false).getAsBoolean()))
            .andThen(stopMotorCommand());
    }

    public Command resetPositionCommand()
    {
        return run ( () -> resetPosition());
    }

    public Command runClimbCommand()
    {
        return run( ()-> runClimb());
    }

    // *****servo*****

    // use this for climb
    public Command setServoPositionCommand(servoPosition pulse)
    {
        return runOnce( ()-> setServoPWM(pulse));
    }

    // public Command setServoToExtendedPositionCommand()
    // {
    //     return runOnce( ()-> setServoPWM(servoPosition.kEXTENDED));
    // }

    // public Command setServoToRetractedPositionCommand()
    // {
    //     return runOnce( ()-> setServoPWM(servoPosition.kRETRACTED));
    // }

    public Command disableServoCommand()
    {
        return runOnce(()-> servo.setDisabled());
    }

    // *****sensor*****
    public BooleanSupplier isDetected()
    {
        return ()-> !getClimbSensor();
    }

    public BooleanSupplier isDetectedAfterDistanceSupplier(double distance, boolean isForward)
    {
        return ()-> getClimbSensorAfterDistance(distance, isForward);
    }
    

    // *** OVERRIDEN METHODS ***!
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        // servo.updateCurPos();
    }

    @Override
    public String toString()
    {
        return "Climb Motor Position = " + getPosition() + ", Servo Position = " + getServoPosition();
    }
}
