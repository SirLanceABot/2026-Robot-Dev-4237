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
        //     climb.setServoPWMCommand(1800)
        // );
        // controller.b().onTrue(
        //     climb.setServoPWMCommand(2100)
        // );
        // controller.y().onTrue(
        //     climb.setServoPWMCommand(1900)
        // );
        // controller.x().onTrue(
        //     climb.setServoPWMCommand(1450)
        // );

        // System.out.println("Servo Pulse Microseconds = " + climb.getServoPWM());

        

        controller.a().onTrue(
            accelerator.setVelocityCommand(0.0)
        );
        controller.x().onTrue(
            accelerator.setVelocityCommand(12.)
        );
        controller.b().onTrue(
            flywheel.setControlVelocityCommand(() -> 59)
        );
        controller.y().onTrue(
            flywheel.setControlVelocityCommand(() -> 0.0)
        );

        // controller.a().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 20)
        // );
        // controller.b().onTrue(
        //     flywheel.stopCommand()
        // );

        System.out.println("flywheel Velocity = " + flywheel.getVelocity() +"duty = " + flywheel.getDutyCycle());

        // System.out.println("accelerator velocity: " + accelerator.getVelocity());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}