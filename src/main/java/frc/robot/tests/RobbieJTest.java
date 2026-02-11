package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoringCommands;
import frc.robot.sensors.HopperCamera;
import frc.robot.sensors.LaserCanSensor;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.sensors.LaserCanSensor;

@SuppressWarnings("unused")
public class RobbieJTest implements Test
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
    private Agitator agitator;
    private Climb climb;
    private Intake intake;
    private Indexer indexer;
    private Accelerator accelerator;
    private Flywheel flywheel; 
    private HopperCamera hopperCamera;
    private LaserCanSensor laserCan;
    
    private final CommandXboxController controller = new CommandXboxController(0);



    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * <p>Modify the {@link frc.robot.TestMode} class to run your test code
     * @param robotContainer The container of all robot components
     */
    public RobbieJTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        agitator = robotContainer.getAgitator();
        intake = robotContainer.getIntake();
        indexer = robotContainer.getIndexer();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        hopperCamera = robotContainer.getHopperCamera();
        climb = robotContainer.getClimb();
        laserCan = robotContainer.getLaserCanSensor();

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
        // controller.x().onTrue(ScoringCommands.IntakeAndScoreCommand(intake, agitator, indexer, accelerator, flywheel));
        // controller.a().onTrue(ScoringCommands.StopIntakeAndScoreCommand(intake, agitator, indexer, accelerator, flywheel));
        // controller.b().onTrue(ScoringCommands.IntakeAndScoreCommandTheSequal(intake, agitator, indexer, accelerator, flywheel));
        // controller.x().onTrue(agitator.forwardCommand());
        // controller.a().onTrue(agitator.reverseCommand());
        // controller.b().onTrue(agitator.stopCommand());
        // controller.x().onTrue(climb.extendToL1Command());
        // controller.a().onTrue(climb.retractFromL1Command());
        // controller.b().onTrue(climb.stopCommand());
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // System.err.println("isYellow:" + hopperCamera.isHoppperFullSupplier().getAsBoolean());
        // System.out.println("Distance: "+ LaserCanSensor.d);
        // System.out.println(climb.getForwardHardLimit());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
