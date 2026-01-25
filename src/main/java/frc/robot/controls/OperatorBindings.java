package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;

public final class OperatorBindings {

    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    
    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    //Variables should be private and static
    private static CommandXboxController controller;

    private static Accelerator accelerator;
    private static Agitator agitator;
    private static Climb climb;
    private static Drivetrain drivetrain;
    private static Flywheel flywheel;
    private static Indexer indexer;
    private static Intake intake;
    private static PoseEstimator poseEstimator;

    private static DoubleSupplier leftYAxis;
    private static DoubleSupplier leftXAxis;
    private static DoubleSupplier rightXAxis;
    private static DoubleSupplier rightYAxis;

    private static int rumbleTime = 0;



    // *** CLASS CONSTRUCTOR ***
    private OperatorBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        controller = robotContainer.getOperatorController();
        accelerator = robotContainer.getAccelerator();
        agitator = robotContainer.getAgitator();
        climb = robotContainer.getClimb();
        drivetrain = robotContainer.getDrivetrain();
        flywheel = robotContainer.getFlywheel();
        indexer = robotContainer.getIndexer();
        intake = robotContainer.getIntake();
        poseEstimator = robotContainer.getPoseEstimator();

        if(controller != null)
        {
            System.out.println("  Constructor Started:  " + fullClassName);

            configSuppliers();

            configAButton();
            configBButton();
            configXButton();
            configYButton();
            configLeftBumper();
            configRightBumper();
            configBackButton();
            configStartButton();
            configLeftTrigger();
            configRightTrigger();
            configLeftStick();
            configRightStick();
            configDpadUp();
            configDpadDown(); 
            configDpadLeft();
            configDpadRight();
            
            configRumble(rumbleTime);
            configDefaultCommands();

            System.out.println("  Constructor Finished: " + fullClassName);
        }
    }

    private static void configSuppliers()
    {
        leftYAxis = () -> -controller.getRawAxis(1);
        leftXAxis = () -> -controller.getRawAxis(0);
        rightXAxis = () -> -controller.getRawAxis(4);
    }

    private static void configAButton()
    {
        Trigger aButton = controller.a();
    }


    private static void configBButton()
    {
        Trigger bButton = controller.b();
    }


    private static void configXButton()
    {
        Trigger xButton = controller.x();
    }


    private static void configYButton()
    {
        Trigger yButton = controller.y();
    }


    private static void configLeftBumper()
    {
        Trigger leftBumper = controller.leftBumper();
    }


    private static void configRightBumper()
    {
        Trigger rightBumper = controller.rightBumper();
    }


    private static void configBackButton()
    {
        Trigger backButton = controller.back();
    }


    private static void configStartButton()
    {
        Trigger startButton = controller.start();
    }



    private static void configLeftTrigger()
    {
        Trigger leftTrigger = controller.leftTrigger();
    }


    private static void configRightTrigger()
    {
        Trigger rightTrigger = controller.rightTrigger();
    }


    private static void configLeftStick()
    {
        Trigger leftStick = controller.leftStick();
    }


    private static void configRightStick()
    {
        Trigger rightStick = controller.rightStick();
    }


    private static void configDpadUp()
    {
        Trigger dpadUp = controller.povUp();
    }


    private static void configDpadDown()
    {
        Trigger dpadDown = controller.povDown();

    }

    private static void configDpadLeft()
    {
        Trigger dpadLeft = controller.povLeft();
    }

    private static void configDpadRight()
    {
        Trigger dpadRight = controller.povRight();
    }

    public static void configRumble(int time)
    {
        BooleanSupplier isRumbleTime = () -> Math.abs(DriverStation.getMatchTime() - time) <= 0.5 && DriverStation.isTeleopEnabled();
        Trigger rumble = new Trigger(isRumbleTime);
        
        rumble
        .onTrue( Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
        .onFalse( Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
    }

    private static void configDefaultCommands()
    {
       
    }    
}
