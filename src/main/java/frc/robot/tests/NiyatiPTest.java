package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

@SuppressWarnings("unused")
public class NiyatiPTest implements Test
{
    // String gets full name of class
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
    private Intake intake = null;
    private final CommandXboxController controller = new CommandXboxController(0);


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public NiyatiPTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        // Prefer the Intake from RobotContainer if available
        if (robotContainer != null && robotContainer.getIntake() != null)
        {
            intake = robotContainer.getIntake();
            System.out.println("good intake :)");
        }
        else
        {
            System.out.println("no intake :(");
            intake = new Intake();
        }

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
    {
        // X to pickup
        controller.x().onTrue(intake.pickupFuelCommand());

        // A to eject
        controller.a().onTrue(intake.ejectCoralCommand());

        // B to stop
        controller.b().onTrue(intake.stopCommand());
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // nothing here; commands are done with controller bindings
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        if (intake != null)
        {
            intake.stop();
            System.out.println("intake stopped in exit()");
        }
    } 
}
