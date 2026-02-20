package frc.robot.tests;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.sensors.CANRange;
import frc.robot.sensors.Hopper;
import frc.robot.sensors.HopperCamera;
import frc.robot.sensors.LaserCanSensor;
import frc.robot.sensors.RangerDistanceSensor;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
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
    // private Agitator agitator;
    private Climb climb;
    private Intake intake;
    private Indexigator indexigator;
    private Accelerator accelerator;
    private Flywheel flywheel; 
    private HopperCamera hopperCamera;
    private LaserCanSensor laserCan;
    private RangerDistanceSensor rangerDistanceSensor;
    private CANRange canrange;
    private CANRange canrange1;
    private Hopper hopper;
    private final CommandXboxController controller = new CommandXboxController(0);
    private Debouncer debouncer = new Debouncer(0.5);

    double distance;

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
        // agitator = robotContainer.getAgitator();
        intake = robotContainer.getIntake();
        indexigator = robotContainer.getIndexigator();
        accelerator = robotContainer.getAccelerator();
        flywheel = robotContainer.getFlywheel();
        hopperCamera = robotContainer.getHopperCamera();
        climb = robotContainer.getClimb();
        laserCan = robotContainer.getLaserCanSensor();
        rangerDistanceSensor = robotContainer.getRangerDistanceSensor();
        // canrange = robotContainer.getCANrange(0); 
        // canrange1 = robotContainer.getCANrange(1); 
        hopper = robotContainer.getHopper();


        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private BooleanSupplier isHopperFullSupplier()
    {
        return () -> (canrange.isBallDetected(29.5) && canrange1.isBallDetected(29.5));
    }

    //     private BooleanSupplier isHopperFullSupplier()
    // {
    //     if((canrange.isBallDetected(24.0)) && (canrange1.isBallDetected(24.0)))
    //             {
    //                 if (timer.get() == 0)
    //                 {
    //                     timer.start();
    //                 }
    //                 else if(timer.get() >= 0.5)
    //                 {
    //                     return ()-> true;
    //                 }
    //             }
    //             else
    //             {
    //                 timer.reset();
    //                 return () -> false;
    //             }
    // }
        

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
        // System.out.println(canrange.toString());
        // controller.x().onTrue(GeneralCommands.intakeUntilFullCommand());
        // controller.b().onTrue(intake.stopCommand());
        // controller.a().onTrue(indexigator.stopCommand());
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // System.err.println("isYellow:" + hopperCamera.isHoppperFullSupplier().getAsBoolean());
        // SmartDashboard.putNumber("Short End sensor", canrange.getDistanceMeters());
        // SmartDashboard.putNumber("Long End sensor", canrange1.getDistanceMeters());

        System.out.println( "is Hopper close: " + hopper.isHopperClosed().getAsBoolean());
        // System.out.println( "is Hopper full: " + hopper.isHopperFullSupplier().getAsBoolean());

        // System.out.println(isHopperFullSupplier());
        // System.out.println(debouncer.calculate(isHopperFullSupplier().getAsBoolean()));
        // System.out.println(canrange.getDistanceMeters());
        // System.out.println(climb.getForwardHardLimit());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
