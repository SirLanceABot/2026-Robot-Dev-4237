package frc.robot.elastic;

import java.lang.invoke.MethodHandles;

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

    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instances variables here

    // private static Field2d autofield = new Field2d();
    // private static Field2d field = new Field2d();

    // private static Trajectory trajectory;

    private static Camera intakeCamera;
    private static Camera shooterCamera;
    private static HopperCamera hopperCamera;
    private static LEDs leds;
    private static Drivetrain drivetrain;
    private static boolean useFullRobot;

    // private static Alert autoAlert = new Alert("Invalid Auto", AlertType.kWarning);
    private static Alert useFullRobotAlert = new Alert("NOT using Full Robot!", AlertType.kError);
    private static Alert autoAlert = new Alert("INVALID AUTO", AlertType.kWarning);
    // private static boolean useFullRobot;

    // StartUp Alerts 
    private static final Alert lowVoltageAlert = new Alert("Battery voltage is LOW", AlertType.kError);
    private static final Alert gyroAlert = new Alert("Gyro is NOT zeroed", AlertType.kWarning);
    private static final Alert canRangeAlert = new Alert("CANRange sensor offline", AlertType.kError);
    private static final Alert cameraAlert = new Alert("Camera not responding", AlertType.kWarning);
    private static final Alert swerveAlert = new Alert("Swerve modules misaligned", AlertType.kWarning);

    private ElasticLance()
    {}

    public static void configElastic(RobotContainer robotContainer)
    {
        intakeCamera    = robotContainer.getIntakeCamera();
        shooterCamera   = robotContainer.getShooterCamera();
        hopperCamera    = robotContainer.getHopperCamera();
        leds            = robotContainer.getLEDs();
        drivetrain      = robotContainer.getDrivetrain();
        useFullRobot    = robotContainer.useFullRobot();
    }

    public static void sendDataToSmartDashboard()
    {
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.00);
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
        
        if(drivetrain != null)
            SmartDashboard.putNumber("Gyro Rotation", drivetrain.getPigeon2().getYaw().getValueAsDouble());

        // updateValidAutoBox();
        updateAllianceColorBox();
        updateHubTagBox();
        updateClimbTagBox();
        updateLEDColorBox();

        if(!useFullRobot && DriverStation.isDisabled())
        {
            useFullRobotAlert.set(true);
            leds.setColorSolidCommand(100, Color.kRed).ignoringDisable(true).schedule();        
        }
    }

    public static void updateValidAutoBox()
    {
        if(DriverStation.isDisabled())
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

    public static void updateStartupAlerts()
    {
        StartUpState state = StartUpCommands.getCurrentState();

        lowVoltageAlert.set(false);
        gyroAlert.set(false);
        canRangeAlert.set(false);
        cameraAlert.set(false);
        swerveAlert.set(false);

        if(state != null)
        {
            switch (state)
            {
                case LOW_VOLTAGE:
                    lowVoltageAlert.set(true);
                    break;

                case GYRO_NOT_ZEROED:
                    gyroAlert.set(true);
                    break;

                case CANRANGE_OFF:
                    canRangeAlert.set(true);
                    break;
                
                case CAMERAS_OFF:
                    cameraAlert.set(true);
                    break;

                case SWERVE_MISALIGNED:
                    swerveAlert.set(true);
                    break;

                case READY:
                    break;
                
                default:
                    break;
            }
        }

    }

}