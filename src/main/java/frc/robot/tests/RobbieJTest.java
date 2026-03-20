package frc.robot.tests;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.subsystems.LEDs;
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
    private LEDs leds = null;
    double distance;
    LEDs.LEDView leftView;
    LEDs.LEDView rightView;

    LEDs.LEDView view1;
    LEDs.LEDView view2;
    LEDs.LEDView view3;
    LEDs.LEDView view4;
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
        // // agitator = robotContainer.getAgitator();
        // intake = robotContainer.getIntake();
        // indexigator = robotContainer.getIndexigator();
        // accelerator = robotContainer.getAccelerator();
        // flywheel = robotContainer.getFlywheel();
        // hopperCamera = robotContainer.getHopperCamera();
        // climb = robotContainer.getClimb();
        // laserCan = robotContainer.getLaserCanSensor();
        // rangerDistanceSensor = robotContainer.getRangerDistanceSensor();
        // // canrange = robotContainer.getCANrange(0); 
        // // canrange1 = robotContainer.getCANrange(1); 
        // hopper = robotContainer.getHopper();
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
        controller.a().onTrue(leds.setFranksThingyCommand(90, Color.kRed));
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // System.out.println(hopper.isHopperFullSupplier().getAsBoolean());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}
