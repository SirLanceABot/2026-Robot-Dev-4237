package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.ColorPattern;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;

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
    private final Agitator agitator;
    private final LEDs leds;
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
        agitator = robotContainer.getAgitator();
        leds = robotContainer.getLEDs();
        // accelerator = robotContainer.getAccelerator();


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
        controller.x().onTrue(
           GeneralCommands.superCoolAutomatedL1ClimbCommandToScoreManyPoints()
        );

        // controller.y().onTrue(
        //     GeneralCommands.ejectFuelInIntakeCommand()
        // );

        // controller.b().onTrue(
        //     GeneralCommands.stopEjectingFuelInIntakeCommand()
        // );

        // controller.a().onTrue(
        //     GeneralCommands.resetIntakeCommand()
        // );

        // controller.b().onTrue(
        //     leds.setColorSolidCommand(100, Color.kRed)
        // );



        // controller.y().onTrue(
        //     GeneralCommands.setLEDCommand(ColorPattern.kSolid, Color.kGreen)
        // );
       
        // controller.b().onTrue(
        //     flywheel.stopCommand()
        // );



        // if(flywheel.isAtSetSpeed(1.0, 1.0).getAsBoolean())
        // {
        //     System.out.println("AT TARGET SPEED**********************************");
        // }
        // else
        // {
        //     System.out.println("NOT AT TARGET SPEED------------------------------");
        // }

        // System.out.println("***************velocity = " + flywheel.getVelocity());

        // flywheel.displayStatorCurrent();
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}