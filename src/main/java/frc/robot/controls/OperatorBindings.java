package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.security.GeneralSecurityException;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.logging.Handler;

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
import frc.robot.commands.GeneralCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Accelerator;
// import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexigator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs.ColorPattern;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Climb.servoPosition;

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
    // private static Agitator agitator;
    private static Climb climb;
    private static Drivetrain drivetrain;
    private static Flywheel flywheel;
    private static Indexigator indexigator;
    private static Intake intake;
    private static PoseEstimator poseEstimator;

    private static DoubleSupplier leftYAxis;
    private static DoubleSupplier leftXAxis;
    private static DoubleSupplier rightXAxis;
    private static DoubleSupplier rightYAxis;

    private static int rumbleTime = 3;


    // *** CLASS CONSTRUCTOR ***
    private OperatorBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        controller = robotContainer.getOperatorController();
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
    }

    // TODO test all bindings

    private static void configAButton()
    {
        Trigger aButton = controller.a();

        // A ~ eject intake
        // aButton.onTrue(GeneralCommands.ejectFuelInIntakeCommand());

        // aButton.onFalse(GeneralCommands.stopEjectingFuelInIntakeCommand());

        aButton.onTrue(intake.turnOnRollersAtSetSpeedCommand(0.5));
    }


    private static void configBButton()
    {
       Trigger bButton = controller.b();

       // B ~ turn off intake
       bButton.onTrue(GeneralCommands.stopIntakingCommand());
    }

    private static void configXButton()
    {
        Trigger xButton = controller.x();

        // X ~ get flywheel to speed
        xButton.onTrue(GeneralCommands.getFlywheelToSpeedCommand());
    }

    private static void configYButton()
    {
        Trigger yButton = controller.y();

        yButton.whileTrue(GeneralCommands.ejectAllFuelSlowlyCommand());

        yButton.onFalse(GeneralCommands.stopEjectingAllFuelCommand());
    }


    private static void configLeftBumper()
    {
        Trigger leftBumper = controller.leftBumper();

        // Left Bumper ~ extend and turn intake on
        leftBumper.onTrue(GeneralCommands.intakeAndTellUsIfItsFullAndKeepGoingCommand());
    }

    private static void configRightBumper()
    {
       Trigger rightBumper = controller.rightBumper();

       // Right Bumper ~ retract intake and turn it off
       rightBumper.onTrue(GeneralCommands.resetIntakeCommand());
    }

    private static void configBackButton()
    {
        Trigger backButton = controller.back();

        // Back ~ kill shooter (stop flywheel/agitator/accelerator)
        backButton.onTrue(GeneralCommands.stopShootingCommand());
    }


    private static void configStartButton()
    {
        Trigger startButton = controller.start();     
        
        // startButton.onTrue(GeneralCommands.unjamIntakeCommand());
        //TODO test this binding
        startButton.whileTrue(GeneralCommands.agitateIntakeCommand().repeatedly());
    }


    // TESTED and WORKS (need to add climb)
    private static void configLeftTrigger()
    {
        Trigger leftTrigger = controller.leftTrigger();

        // Left Trigger ~ left auto climb
        // leftTrigger.whileTrue(new DeferredCommand(() -> ScoringCommands.autoClimbCommand(drivetrain, poseEstimator, climb, () -> true), Set.of()));   
        
        leftTrigger.onTrue(GeneralCommands.intakeCommand());
    }

    // TESTED and WORKS (need to add climb)
    private static void configRightTrigger()
    {
        Trigger rightTrigger = controller.rightTrigger();

        // Right Trigger ~ right auto climb
        // rightTrigger.whileTrue(new DeferredCommand(() -> ScoriDngCommands.autoClimbCommand(drivetrain, poseEstimator, climb, () -> false), Set.of()));
        // rightTrigger.onTrue(GeneralCommands.intakeAndTellUsIfItsFullAndKeepGoingCommand());

        rightTrigger.onTrue(GeneralCommands.intakeDepotCommand());    
    }


    private static void configLeftStick()
    {
        Trigger leftStick = controller.leftStick();

        // leftStick.onTrue(climb.manualMoveClimbDownCommand());

        // leftStick.onFalse(climb.stopMotorCommand());
    }


    private static void configRightStick()
    {
        Trigger rightStick = controller.rightStick();

        // rightStick.onTrue(climb.manualMoveClimbUpCommand());

        // rightStick.onFalse(climb.stopMotorCommand());
    }

    // TODO test binding and command
    private static void configDpadUp()
    {
        Trigger dpadUp = controller.povUp();

        if(climb != null)
        {
             // dpad Up ~ manual extend climb
            // dpadUp.onTrue(GeneralCommands.extendClimbToL1Command());
            // dpadUp.onTrue(climb.manualMoveClimbUpCommand());

            // dpadUp.onFalse(climb.stopMotorCommand());
        }
    }

    // TODO test binding and command
    private static void configDpadDown()
    {
        Trigger dpadDown = controller.povDown();

        if(climb != null)
        {
            // dpad Down ~ mantual retract climb
            // dpadDown.onTrue(GeneralCommands.ascendFromL1Command());
            // dpadDown.onTrue(climb.manualMoveClimbDownCommand());

            // dpadDown.onFalse(climb.stopMotorCommand());
        }
    }

    private static void configDpadLeft()
    {
        Trigger dpadLeft = controller.povLeft();

        if(climb != null)
        {
            // dpad Left ~ move servo to start position
            // dpadLeft.onTrue(climb.setServoPositionCommand(servoPosition.kRETRACTED));
        }
    }

    private static void configDpadRight()
    {
        Trigger dpadRight = controller.povRight();

        if(climb != null)
        {
            // dpad Right ~ move servo to climb position
            // dpadRight.onTrue(climb.setServoPositionCommand(servoPosition.kEXTENDED));
        }
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
