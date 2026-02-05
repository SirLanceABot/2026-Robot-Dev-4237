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
import frc.robot.sensors.Camera;
import frc.robot.sensors.HopperCamera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;


public class ElasticLance 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static Color allianceColor = new Color();
    // Come up with a color indicator for elastic and leds that matches shoot on the move range
    // private static Color validTagColor = new Color();

    private static Color validAutoColor = new Color();

    private static Color LEDColor = new Color();

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

    // private static boolean useFullRobot;

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
        SmartDashboard.putNumber("Gyro Rotation", drivetrain.getPigeon2().getYaw().getValueAsDouble());

        // updateValidAutoBox();
        // updateAllianceColorBox();
        // updateLEDColorBox();

        if(!useFullRobot && DriverStation.isDisabled())
        {
            useFullRobotAlert.set(true);
            leds.setColorSolidCommand(100, Color.kRed).ignoringDisable(true).schedule();        }
    }

    // public static void updateValidAutoBox()
    // {
    //     if(DriverStation.isDisabled())
    //     {

    //     }
    // }

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
    }

    public static void updateLEDColorBox()
    {
        LEDColor = LEDs.getColor();
    }

    // figure out a way to get the current color from the led class

}