package frc.robot.loggers;

import static frc.robot.Constants.NetworkTableLance.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public final class DataLogFile 
{
    private static NetworkTableInstance networkTableInstance;
    private static DataLog dataLog;

    private DataLogFile()
    {}

    public static void config()
    {
        // DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        dataLog = DataLogManager.getLog();
        
        networkTableInstance = NetworkTableInstance.getDefault();
        DriverStation.startDataLog(dataLog, true);
        networkTableInstance.startEntryDataLog(dataLog, "/FMSInfo", "NT:/FMSInfo");
        networkTableInstance.startEntryDataLog(dataLog, "/" + TEAM_TABLE, "NT:/" + TEAM_TABLE);
        networkTableInstance.startEntryDataLog(dataLog, "/" + ADVANTAGE_SCOPE_TABLE, "NT:/" + ADVANTAGE_SCOPE_TABLE);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + CAMERA_1_BOT_POSE, "NT:/" + CAMERA_1_BOT_POSE);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + CAMERA_2_BOT_POSE, "NT:/" + CAMERA_2_BOT_POSE);
        // networkTableInstance.startEntryDataLog(dataLog, "/SmartDashboard", "NT:/SmartDashboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/Shuffleboard", "NT:/Shuffleboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/LiveWindow", "NT:/LiveWindow");
        networkTableInstance.startConnectionDataLog(dataLog, "NTConnection");
    }
}
