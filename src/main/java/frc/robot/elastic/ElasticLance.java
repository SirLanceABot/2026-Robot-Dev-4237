package frc.robot.elastic;

import java.lang.invoke.MethodHandles;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.RobotContainer;
import frc.robot.pathplanner.PathPlannerLance;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Hopper;
import frc.robot.sensors.HopperCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

import frc.robot.commands.StartUpCommands;
import frc.robot.commands.StartUpCommands.StartUpState;

public class ElasticLance 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static Color allianceColor = new Color();
    // Come up with a color indicator for elastic and leds that matches shoot on the move range
    // private static Color validTagColor = new Color();

    private static Color validAutoColor = new Color();

    private static Color LEDColor = new Color();
    private static Color hubTagColor = new Color();
    private static Color climbTagColor = new Color();
    private static Color hopperColor = new Color();

    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instances variables here

    // private static Field2d autofield = new Field2d();
    // private static Field2d field = new Field2d();

    // private static Trajectory trajectory;

    // private static Camera intakeCamera;
    private static Camera shooterCamera;
    // private static HopperCamera hopperCamera;
    private static LEDs leds;
    private static Drivetrain drivetrain;
    private static boolean useFullRobot;
    private static Hopper hopper;

    private static Alert useFullRobotAlert = new Alert("NOT using Full Robot!", AlertType.kError);
    private static Alert autoAlert = new Alert("INVALID AUTO", AlertType.kWarning);
    // private static boolean useFullRobot;

    // StartUp Alerts 
    private static final Alert lowVoltageAlert = new Alert("Battery voltage is LOW", AlertType.kError);
    // private static final Alert gyroAlert = new Alert("Gyro is NOT zeroed", AlertType.kWarning);
    private static final Alert leftCANRangeAlert = new Alert("Left CANRange sensor offline", AlertType.kError);
    private static final Alert rightCANRangeAlert = new Alert("Right CANRange sensor offline", AlertType.kError);
    private static final Alert swerveAlert = new Alert("Swerve modules misaligned", AlertType.kWarning);
    private static final Alert operatorControllerAlert = new Alert("Operator - press Start Button", AlertType.kWarning);
    private static final Alert driverControllerAlert = new Alert("Driver - press Back Button", AlertType.kWarning);

    private ElasticLance()
    {}

    public static void configElastic(RobotContainer robotContainer)
    {
        shooterCamera   = robotContainer.getShooterCamera();
        leds            = robotContainer.getLEDs();
        drivetrain      = robotContainer.getDrivetrain();
        useFullRobot    = robotContainer.useFullRobot();
        hopper          = robotContainer.getHopper();

        driverControllerAlert.set(true);
        operatorControllerAlert.set(true);
        swerveAlert.set(true);
        rightCANRangeAlert.set(true);
        leftCANRangeAlert.set(true);
        lowVoltageAlert.set(true);
    }

    public static void sendDataToSmartDashboard()
    {
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.00);
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
        
        if(drivetrain != null)
        {
            SmartDashboard.putNumber("Gyro Rotation", drivetrain.getPigeon2().getYaw().getValueAsDouble());
            SmartDashboard.putNumber("Rotation", drivetrain.getState().Pose.getRotation().getDegrees());

            // SmartDashboard.putData("Swerve Drive", new Sendable() {
            //     @Override
            //     public void initSendable(SendableBuilder builder) {
            //         builder.setSmartDashboardType("SwerveDrive");

            //         builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getModule(12).getCurrentState().angle.getDegrees(), null);
            //         builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getModule(10).getCurrentState().speedMetersPerSecond, null);

            //         builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getModule(9).getCurrentState().angle.getDegrees(), null);
            //         builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getModule(7).getCurrentState().speedMetersPerSecond, null);

            //         builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(6).getCurrentState().angle.getDegrees(), null);
            //         builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getModule(4).getCurrentState().speedMetersPerSecond, null);

            //         builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getModule(3).getCurrentState().angle.getDegrees(), null);
            //         builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);

            //         builder.addDoubleProperty("Robot Angle", () -> drivetrain.getRotation3d().getAngle(), null);

            //     }
            // });
        }

        updateAllianceColorBox();
        updateHubTagBox();
        updateClimbTagBox();
        updateLEDColorBox();
        updateValidAutoBox();
        updateStartupAlerts();
        updateHopperFullBox();

        if(!useFullRobot && DriverStation.isDisabled())
        {
            useFullRobotAlert.set(true);
        }
    }

    public static void updateValidAutoBox()
    {
        if(DriverStation.isDisabled() && AutoBuilder.isConfigured())
        {
            if(PathPlannerLance.getAutonomousCommand().getName().equalsIgnoreCase("InstantCommand"))
            {
                validAutoColor = Color.kRed;
                autoAlert.set(true);
            }
            else
            {
                validAutoColor = Color.kGreen;
                autoAlert.set(false);
            }
        }
        SmartDashboard.putString("Is Auto Valid", validAutoColor.toHexString());
    }
    
    public static void updateHubTagBox()
    {
        int tagID = 0;

        if(shooterCamera != null)
        {
            tagID = (int) shooterCamera.getTagId();
        }

        if(shooterCamera == null)
        {
            hubTagColor = Color.kGray;
        }
        else if((tagID >= 8 && tagID <= 11) || (tagID >= 24 && tagID <= 27))
        {
            hubTagColor = Color.kGreen;
        }
        else 
        {
            hubTagColor = Color.kRed;
        }

        SmartDashboard.putString("Is hub tag", hubTagColor.toHexString());

    }

    public static void updateClimbTagBox()
    {
        int tagID = 0;

        if(shooterCamera != null)
        {
            tagID = (int) shooterCamera.getTagId();
        }

        if(shooterCamera == null)
        {
            climbTagColor = Color.kGray;
        }
        else if(tagID == 16 || tagID == 15 || tagID == 31 || tagID == 32)
        {
            climbTagColor = Color.kGreen;
        }
        else 
        {
            climbTagColor = Color.kRed;
        }

        SmartDashboard.putString("Is climb tag", climbTagColor.toHexString());

    }

    public static void updateAllianceColorBox()
    {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            allianceColor = Color.kRed;
        }
        else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
            allianceColor = Color.kBlue;
        }
        else
        {
            allianceColor = Color.kGray;
        }

        SmartDashboard.putString("Alliance Color", allianceColor.toHexString());
    }

    public static void updateLEDColorBox()
    {
        LEDColor = LEDs.getColor();

        SmartDashboard.putString("LED Color", LEDColor.toHexString());
    }

    public static void updateHopperFullBox()
    {
        if(hopper != null)
        {
            if(hopper.isHopperFullSupplier().getAsBoolean())
            {
                hopperColor = Color.kGreen;
            }
            else
            {
                hopperColor = Color.kYellow;
            }
        }
        else
        {
            hopperColor = Color.kRed;
        }

        SmartDashboard.putString("Hopper Color", hopperColor.toHexString());
    }

    public static void updateStartupAlerts()
    {
        StartUpState state = StartUpCommands.getCurrentState();

        if(state != null)
        {
            switch (state)
            {
                case LOW_VOLTAGE:
                    lowVoltageAlert.set(true);
                    break;

                // case GYRO_NOT_ZEROED:
                //     gyroAlert.set(true);
                //     lowVoltageAlert.set(false);
                //     break;

                case LEFT_CAN_RANGE_OFF:
                    leftCANRangeAlert.set(true);
                    lowVoltageAlert.set(false);
                    break;
                
                case RIGHT_CAN_RANGE_OFF:
                    rightCANRangeAlert.set(true);
                    leftCANRangeAlert.set(false);
                    break;

                case SWERVE_MISALIGNED:
                    swerveAlert.set(true);
                    rightCANRangeAlert.set(false);
                    break;

                case OPERATOR_CONTROLLER_OFF:
                    operatorControllerAlert.set(true);
                    swerveAlert.set(false);
                    break;

                case DRIVER_CONTROLLER_OFF:
                    driverControllerAlert.set(true);
                    operatorControllerAlert.set(false);
                    break;

                case READY:
                    driverControllerAlert.set(false);
                    break;
                
                default:
                    break;
            }
        }

    }

}