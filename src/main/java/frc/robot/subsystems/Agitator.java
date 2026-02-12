// package frc.robot.subsystems;

// import static frc.robot.Constants.Agitator.*;

// import java.lang.invoke.MethodHandles;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.motors.SparkMaxLance;
// import frc.robot.motors.TalonFXLance;

// /**
//  * Creates an agitator
//  */
// public class Agitator extends SubsystemBase
// {
//     // This string gets the full name of the class, including the package name
//     private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

//     // *** STATIC INITIALIZATION BLOCK ***
//     // This block of code is run first when the class is loaded
//     static
//     {
//         System.out.println("Loading: " + fullClassName);
//     }
    

//     // *** INNER ENUMS and INNER CLASSES ***
//     // Put all inner enums and inner classes here


    
//     // *** CLASS VARIABLES & INSTANCE VARIABLES ***
//     // Put all class variables and instance variables here
//     private final TalonFXLance motor = new TalonFXLance(MOTOR, MOTOR_CAN_BUS, "Agitator Motor ");
//     // private final SparkMaxLance agitatorMotor = new SparkMaxLance(3, "ROBORIO", "Agitator");

//     //PID
//     private final double kP = 0.3;
//     private final double kI = 0.0;
//     private final double kD = 0.0;

//     private final double AGITATOR_DIAMETER_FEET = 4237.0;
//     private final double GEAR_RATIO = 1.0 / 1.0;
//     private final double VELOCITY_CONVERSION_FACTOR = (Math.PI * AGITATOR_DIAMETER_FEET) / GEAR_RATIO; // rev/s to ft/s using gear ratio // not checked

//     // *** CLASS CONSTRUCTORS ***
//     // Put all class constructors here

//     /** 
//      * Creates a new agitator. 
//      */
//     public Agitator()
//     {
//         super("Agitator Subsystem");
//         System.out.println("  Constructor Started:  " + fullClassName);

//         configMotors();

//         System.out.println("  Constructor Finished: " + fullClassName);
//     }


//     // *** CLASS METHODS & INSTANCE METHODS ***
//     // Put all class methods and instance methods here

//     private void configMotors()
//     {
//         motor.setupFactoryDefaults();
//         motor.setupPIDController(0, kP, kI, kD);
//         motor.setupCoastMode();
//         // motor.setupVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR); // rev/s to ft/s
//     }

//     private void set(double speed)
//     {
//         motor.set(speed);
//     }

//     /**
//      * This sets the speed of the motors.
//      * @param speed The motor speed 
//      */
//     private void setControlVelocity(double speed)
//     {
//         // motor.set(speed);
//         motor.setControlVelocity(speed);
//     }

//     public void stop()
//     {
//         setControlVelocity(0.0);
//     }

//     /**
//      * 
//      */
//     public Command forwardCommand()
//     {
//         return run( () -> setControlVelocity(50.0));
//     }

//     /**
//      * 
//      */
//     public Command reverseCommand(DoubleSupplier speed)
//     {
//         return run( () -> setControlVelocity(50.0));
//     }

//     // Use a method reference instead of this method
//     public Command stopCommand()
//     {
//         // return run( () -> stop() );
//         return runOnce(this::stop);
//     }


//     // *** OVERRIDEN METHODS ***
//     // Put all methods that are Overridden here

//     @Override
//     public void periodic()
//     {
//         // This method will be called once per scheduler run
//         // Use this for sensors that need to be read periodically.
//         // Use this for data that needs to be logged.
//     }

//     @Override
//     public String toString()
//     {
//         return "";
//     }
// }
