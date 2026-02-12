package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;

@SuppressWarnings("unused")
public class LoganBTest implements Test
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
    private final Accelerator acceleartor;
    // private final Agitator agitator;
    private final Indexigator indexigator;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public LoganBTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        this.flywheel = robotContainer.getFlywheel();
        this.acceleartor = robotContainer.getAccelerator();
        // this.agitator = robotContainer.getAgitator();
        this.indexigator = robotContainer.getIndexigator();

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
        //     ScoringCommands.shootFromStandstillCommand(agitator, acceleartor, flywheel)
        // );
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
