package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Intake;

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
    private final Accelerator accelerator;
    private final Intake intake;
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
        this.accelerator = robotContainer.getAccelerator();
        this.intake = robotContainer.getIntake();
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

        // controller.a().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 41.0)
        // );
        // controller.x().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 39.0)
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
        //     indexigator.setForwardCommand()
        // );
        // controller.b().onTrue(
        //     indexigator.stopCommand()
        // );
        // controller.x().onTrue(
        //     accelerator.setVelocityCommand(12.0)
        // );
        // controller.y().onTrue(
        //     flywheel.setControlVelocityCommand(() -> 25)
        // );
        // controller.start().onTrue(
        //     flywheel.stopCommand()
        // );
        // controller.back().onTrue(
        //     accelerator.stopCommand()
        // );

        controller.a().onTrue(
            GeneralCommands.intakeCommand()
        );
        // controller.b().whileTrue(
        //     intake.moveIntakeInCommand()
        // );
        // controller.b().onFalse(
        //     intake.stopCommand()
        // );
        // controller.a().whileTrue(
        //     intake.moveIntakeOutCommand()
        // );
        // controller.a().onFalse(
        //     intake.stopCommand()
        // );
        System.out.println("pivot pos: " + intake.getPivotPosition());
    }
    //9.35888671875

    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
