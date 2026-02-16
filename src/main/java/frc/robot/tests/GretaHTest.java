package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Flywheel;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Climb;

@SuppressWarnings("unused")
public class GretaHTest implements Test
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
    private final Indexigator indexigator;
    private final Accelerator accelerator;
    private final Climb climb;
    // private final Agitator agitator;
    private final Flywheel flywheel;
    private final CommandXboxController controller = new CommandXboxController(0);


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public GretaHTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        this.indexigator = robotContainer.getIndexigator();
        this.accelerator = robotContainer.getAccelerator();
        this.climb = robotContainer.getClimb();
        // this.agitator = robotContainer.getAgitator();
        this.flywheel = robotContainer.getFlywheel();

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
        // controller.a().onTrue(indexigator.setForwardCommand());
        // controller.x().onTrue(indexigator.setBackwardCommand());
        // controller.b().onTrue(indexigator.stopCommand());
        // controller.a().onTrue(climb.extendServoCommand());
        // controller.b().onTrue(climb.retractServoCommand());
        // controller.x().onTrue(climb.setServoPositionCommand(48));
        // controller.y().onTrue(climb.setServoPositionCommand(0.0));

    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // System.out.println(indexer.getVelocity());
        
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
