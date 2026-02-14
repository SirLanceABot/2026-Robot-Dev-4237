package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.ColorPattern;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.motors.LinearServo;

@SuppressWarnings("unused")
public class RobbieFTest implements Test
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
    private final CommandXboxController controller = new CommandXboxController(0);
    
    private final Flywheel flywheel;
    // private final Agitator agitator;
    private final LEDs leds;
    private final Climb climb;
    private final Accelerator accelerator;
    // private final Accelerator accelerator;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public RobbieFTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;

        flywheel = robotContainer.getFlywheel();
        // agitator = robotContainer.getAgitator();
        leds = robotContainer.getLEDs();
        climb = robotContainer.getClimb();
        accelerator = robotContainer.getAccelerator();


        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

        

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {}

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // controller.a().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 45.0)
        // );
        // controller.x().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 48.0)
        // );
        // controller.y().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 70.0)
        // );
        // controller.b().onTrue(
        //     accelerator.onCommand()
        // );
        // controller.start().onTrue(
        //     flywheel.stopCommand()
        // );

        // controller.a().onTrue(
        //     servo.setPosition(5.0) 
        // );
        // controller.b().onTrue(
        //     servo.setPosition(0.0)
        // );

        controller.a().onTrue(
            climb.fullyExtendServoCommand()
        );
        controller.b().onTrue(
            climb.fullyRestractServoCommand()
        );
        controller.y().onTrue(
            climb.setServoPositionCommand(50.0)
        );

        

        // controller.a().onTrue(
        //     accelerator.onCommand()  
        // );
        // controller.x().onTrue(
        //     accelerator.setVelocityCommand(3.0)
        // );
        // controller.b().onTrue(
        //     accelerator.stopCommand()
        // );

        // System.out.println("flywheel Velocity = " + flywheel.getVelocity() + "cycle=" + flywheel.getDutyCycle());

        System.out.println("Servo Position = " + climb.getServoPosition());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}