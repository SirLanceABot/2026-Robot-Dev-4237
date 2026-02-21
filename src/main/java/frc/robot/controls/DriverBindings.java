package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;

public final class DriverBindings {

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
    // private static Agitator agitator;
    private static Climb climb;
    private static Drivetrain drivetrain;
    private static Flywheel flywheel;
    private static Indexigator indexigator;
    private static Intake intake;
    private static LEDs leds;
    private static PoseEstimator poseEstimator;

    private static DoubleSupplier leftYAxis;
    private static DoubleSupplier leftXAxis;
    private static DoubleSupplier rightXAxis;
    private static DoubleSupplier scaleFactorSupplier;
    private static DoubleSupplier lockAngleSupplier;

    private static final double CRAWL_SPEED = 0.225;
    private static final double WALK_SPEED = 0.675;
    private static final double RUN_SPEED = 1.0;
    private static double scaleFactor = 0.5;

    private static int rumbleTime = 3;



    // *** CLASS CONSTRUCTOR ***
    private DriverBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        controller = robotContainer.getDriverController();
        accelerator = robotContainer.getAccelerator();
        // agitator = robotContainer.getAgitator();
        climb = robotContainer.getClimb();
        drivetrain = robotContainer.getDrivetrain();
        flywheel = robotContainer.getFlywheel();
        indexigator = robotContainer.getIndexigator();
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
            configRumble(135);
            configRumble(110);
            configRumble(85);
            configRumble(60);
            configRumble(35);

            configDefaultCommands();

            System.out.println("  Constructor Finished: " + fullClassName);
        }
    }

    private static void configSuppliers()
    {
        leftYAxis = () -> -controller.getRawAxis(1);
        leftXAxis = () -> -controller.getRawAxis(0);
        rightXAxis = () -> -controller.getRawAxis(4);
        scaleFactorSupplier = () -> scaleFactor;
        lockAngleSupplier = () -> Math.PI / 4.0;
    }

    // shoot on move
    private static void configAButton()
    {
        Trigger aButton = controller.a();

        // ANGLE LOCK WORKS ON 2026 DRIVETRAIN FOR BOTH ALLIANCES
        // aButton
        // .whileTrue(drivetrain.angleLockDriveCommand(leftYAxis, leftXAxis, scaleFactorSupplier, () -> (poseEstimator.getAngleToAllianceHub().getAsDouble())));
        
        // drive part of shoot from stanstill works on 2026 drivetrain for both alliances
        // aButton
        // .whileTrue(ScoringCommands.shootFromStandstillCommand(drivetrain, poseEstimator));

        // drive part of shoot on the move works on 2026 drivetrain for both alliances
        // aButton.whileTrue(drivetrain.angleLockDriveCommand(leftYAxis, leftXAxis, scaleFactorSupplier, () -> (poseEstimator.getRotationToCalculatedTarget().getAsDouble())));
        
        aButton
        .whileTrue(
            Commands.parallel(
                drivetrain.angleLockDriveCommand(leftYAxis, leftXAxis, scaleFactorSupplier, () -> (poseEstimator.getRotationToCalculatedTarget().getAsDouble())), // Tested this command in 25 repo, works there
                new DeferredCommand(() -> ScoringCommands.shootOnTheMoveCommand(drivetrain, indexigator, accelerator, flywheel, poseEstimator), Set.of())));
            
        aButton.onFalse(GeneralCommands.stopShootingCommand()); // TODO test this line
    }

    // shoot still
    private static void configBButton()
    {
        Trigger bButton = controller.b();

        bButton.onTrue(ScoringCommands.shootFromStandstillCommand(drivetrain, indexigator, accelerator, flywheel, poseEstimator));

        bButton.onFalse(GeneralCommands.stopShootingCommand());
    }


    private static void configXButton()
    {
        Trigger xButton = controller.x();
    
        // xButton
        // .whileTrue(
        //     Commands.parallel(
        //         drivetrain.angleLockDriveCommand(leftYAxis, leftXAxis, scaleFactorSupplier, () -> poseEstimator.pureLeadingAngle(poseEstimator.getAllianceHubPose()).getAsDouble()),
        //         ScoringCommands.physicsShootOnTheMove(drivetrain, poseEstimator, indexigator, accelerator, flywheel)));

        // xButton.onFalse(GeneralCommands.stopShootingCommand());
    }

    // pass
    private static void configYButton()
    {
        Trigger yButton = controller.y();

        yButton.whileTrue(ScoringCommands.passCommand(indexigator, accelerator, flywheel));

        yButton.onFalse(GeneralCommands.stopShootingCommand());

        // Auto climb testing
        // yButton.whileTrue(
        //     new DeferredCommand(() -> ScoringCommands.autoClimbCommand(drivetrain, poseEstimator, climb, () -> true), Set.of()));
    }


    private static void configLeftBumper()
    {
        Trigger leftBumper = controller.leftBumper();
        //Decreases Drive speed each time you press the left bumper
        leftBumper
        .onTrue(Commands.runOnce(() -> scaleFactor = (scaleFactor > WALK_SPEED ? WALK_SPEED : CRAWL_SPEED)));
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
        startButton
            .onTrue(Commands.runOnce(() -> drivetrain.resetForFieldCentric(), drivetrain));
    }


    private static void configLeftTrigger()
    {
        Trigger leftTrigger = controller.leftTrigger();
        //Increases Drive speed each time you press the leftTrigger
        leftTrigger
        .onTrue(Commands.runOnce(() -> scaleFactor = (scaleFactor < WALK_SPEED ? WALK_SPEED : RUN_SPEED)));
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
        if(drivetrain != null)
        {
            drivetrain.setDefaultCommand(drivetrain.driveCommand(leftYAxis, leftXAxis, rightXAxis, scaleFactorSupplier));     
        }

        //TODO this should be tested, it might cause our LEDs to immediately turn back to red 
        // if(leds != null)
        // {
        //     leds.setDefaultCommand(GeneralCommands.defaultLEDCommand());
        // }
    }    
}
