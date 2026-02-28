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

    // private BooleanSupplier isHopperFullSupplier()
    // {
    //     return () -> (canrange.isBallDetected(29.5) && canrange1.isBallDetected(29.5));
    // }

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
    // LEDs.LEDView leftView = leds.createView(0, 17);
    // LEDs.LEDView rightView = leds.createView(18, 35);
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
        // LEDs.LEDView leftView = leds.createView(0, 17);
        // LEDs.LEDView rightView = leds.createView(18, 35);

        // controller.a().onTrue(leftView.setViewColorSolidCommand(80, Color.kDarkSlateBlue));
        // controller.a().onTrue(leftView.setViewColorGradientCommand(80, false, Color.kRed, Color.kLime));
        // controller.b().onTrue(rightView.setViewColorRainbowCommand(80, false));
        // controller.x().onTrue(leftView.setViewColorGradientCommand(80, true, Color.kRed, Color.kLime));

        

        // controller.x().onTrue(leftView.setViewColorBlinkCommand(40, Color.kHotPink, 0.5));
        // controller.y().onTrue(rightView.setViewColorBreatheCommand(40, Color.kLime, 0.5));
        // controller.back().onTrue(leftView.setOffCommand());
        // controller.start().onTrue(rightView.setOffCommand());
        // controller.a().onTrue(leds.setColorGradientCommand(80, Color.kAzure, Color.kRed, Color.kLime, Color.kIndigo));
        // controller.a().onTrue(leds.setColorSolidCommand(80, Color.kAliceBlue));
        // controller.b().onTrue(leds.setColorSolidCommand(20, Color.kAliceBlue));
        // controller.x().onTrue(leds.offCommand());
        // controller.b().onTrue(leds.setColorRainbowCommand());
        // controller.x().onTrue(leds.setColorBlinkCommand(80, Color.kAliceBlue));
        // controller.y().onTrue(leds.setColorBreatheCommand(80, Color.kBeige));

    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        if(controller.leftBumper().getAsBoolean())
        {
            if(view1 != null && view2 != null && view3 != null && view4 != null)
            {
                leds.deleteView(view1);
                leds.deleteView(view2);
                leds.deleteView(view3);
                leds.deleteView(view4);
            }
            
            leftView = leds.createView(0, 17);
            rightView = leds.createView(18, 35);

            if(controller.a().getAsBoolean())
                leftView.setViewColorGradientCommand(80, true, Color.kRed, Color.kLime).schedule();
            if(controller.b().getAsBoolean())
                rightView.setViewColorRainbowCommand(80, true).schedule();
        }

        if(controller.rightBumper().getAsBoolean())
        {
            if(leftView != null && rightView != null)
            {
                leds.deleteView(leftView);
                leds.deleteView(rightView);
            }

            // leds.setColorSolidCommand(80, Color.kRed);

            view1 = leds.createView(0, 8);
            view2 = leds.createView(9, 17);
            view3 = leds.createView(18, 26);
            view4 = leds.createView(27, 35);

            if(controller.povDown().getAsBoolean())
                view1.setViewColorSolidCommand(80, Color.kLime).schedule();
            if(controller.povUp().getAsBoolean())
                view2.setViewColorSolidCommand(80, Color.kLime).schedule();
            if(controller.povLeft().getAsBoolean())
                view3.setViewColorSolidCommand(80, Color.kLime).schedule();
            if(controller.povRight().getAsBoolean())
                view4.setViewColorSolidCommand(80, Color.kLime).schedule();
        }

        // System.err.println("isYellow:" + hopperCamera.isHoppperFullSupplier().getAsBoolean());
        // SmartDashboard.putNumber("Short End sensor", canrange.getDistanceMeters());
        // SmartDashboard.putNumber("Long End sensor", canrange1.getDistanceMeters());

        // System.out.println( "is Hopper close: " + hopper.isHopperClosed().getAsBoolean());
        // System.out.println("Left: " + hopper.getLimitSwitchLeft());

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
