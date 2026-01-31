package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.subsystems.Intake;

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
    // private Intake intake = null;
    private LEDs leds = null;
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Joystick joystick = new Joystick(0);

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
        // INTAKE STUFF
        // Prefer the Intake from RobotContainer if available
        // if (robotContainer != null && robotContainer.getIntake() != null)
        // {
            // intake = robotContainer.getIntake();
        //     System.out.println("good intake :)");
        // }
        // else
        // {
        //     System.out.println("no intake :(");
        //     intake = new Intake();
        // }

        //LEDS STUFF
        leds = robotContainer.getLEDs();

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
        // INTAKE STUFF
        // // X to pickup
        // controller.x().onTrue(intake.pickupFuelCommand()).debounce(0.1);

        // // A to eject
        // controller.a().onTrue(intake.ejectFuelCommand()).debounce(0.1);

        // // B to stop
        // controller.b().onTrue(intake.stopCommand()).debounce(0.1);
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        controller.x().onTrue(leds.setColorSolidCommand(20, Color.kRed));
        controller.a().onTrue(leds.setMovingRainbowCommand()); //is there no way to add a .schedule?

        // if(joystick.getRawButton(1))
        // {
        //     leds.setColorSolidCommand(60, Color.kBlue).schedule();
        // }
        // else if (joystick.getRawButton(2))
        // {
        //     // leds.setColorRainbowCommand();
        //     leds.setColorSolidCommand(60, Color.kRed).schedule();
        // }
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        // if (intake != null)
        // {
        //     intake.stop();
        //     System.out.println("intake stopped in exit()");
        // }
    } 
}
