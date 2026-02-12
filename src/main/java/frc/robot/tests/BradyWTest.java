package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.motors.SparkMaxLance;
import frc.robot.subsystems.Accelerator;

@SuppressWarnings("unused")
public class BradyWTest implements Test
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



    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    private final RobotContainer robotContainer;
    // private SparkMaxLance motor;
    private Accelerator accelerator;
    private CommandXboxController controller;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public BradyWTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        accelerator = robotContainer.getAccelerator();
        controller = new CommandXboxController(0);

        // configMotors();
        bindings();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    // private void configMotors()
    // {
    //     accelerator.setupFactoryDefaults();
    //     accelerator.setupBrakeMode();
    // }

    private void bindings()
    {
        controller.a().onTrue(accelerator.onCommand());
        controller.x().onTrue(accelerator.reverseCommand());
        controller.b().onTrue(accelerator.stopCommand());
        
        controller.start().onTrue(accelerator.setVelocityCommand(2.0));
        controller.back().onTrue(accelerator.setVelocityCommand(-2.0));
    }
    

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {

    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        System.out.println(accelerator);
        System.out.println("Accelerator Position: " + accelerator.getPosition());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
