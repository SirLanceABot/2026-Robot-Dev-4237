package frc.robot.sensors;

import static frc.robot.Constants.NetworkTableLance.*;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Camera
 */
public class Camera extends SubsystemBase
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



    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private String cameraName;
    private LimelightHelpers.PoseEstimate poseEstimate;
    // private double[] poseArray = new double[3];

    // private DoubleEntry yawEntry;
    // private DoubleArrayEntry poseEntry;
    private NetworkTable ASTable;
    private StructPublisher<Pose2d> cameraEntry;
    private Pose2d cameraPose;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Camera. 
     */
    public Camera(String cameraName)
    {   
        this.cameraName = cameraName;
        System.out.println("  Constructor Started:  " + fullClassName + ">>" + cameraName);

        ASTable = NetworkTableInstance.getDefault().getTable(ADVANTAGE_SCOPE_TABLE);
        cameraEntry = ASTable.getStructTopic(cameraName + " pose", Pose2d.struct).publish();
        // poseEntry = ASTable.getDoubleArrayTopic(cameraName + " pose").getEntry(new double[3]);
        // yawEntry = ASTable.getDoubleTopic("GyroYaw").getEntry(0.0);

        System.out.println("  Constructor Finished: " + fullClassName + ">>" + cameraName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    /**
     * Returns the value of the sensor
    * @return The value of periodData.sensorValue
    */
    public Pose2d getPose()
    {
        if(poseEstimate != null)
        {
            return poseEstimate.pose;
        }
        else
        {
            return null;
        }
    }

    public double getTimestamp()
    {
        if(poseEstimate != null)
        {
            return Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds);
        }
        else
        {
            return -1.00;
        }
    }

    public int getTagCount()
    {
        if(poseEstimate != null)
        {
            return poseEstimate.tagCount;
        }
        else
        {
            return -1;
        }
    }

    public double avgTagDistance()
    {
        if(poseEstimate != null)
        {
            return poseEstimate.avgTagDist;
        }
        else
        {
            return -1.00;
        }
    }

    public double getTagId()
    {
        return LimelightHelpers.getFiducialID(cameraName);
    }

    public double getTX()
    {
        return LimelightHelpers.getTX(cameraName);
    }

     public double getTY()
    {
        return LimelightHelpers.getTY(cameraName);
    }   

    public double getTA()
    {
        return LimelightHelpers.getTA(cameraName);
    }

    public boolean isValidTagInFrame()
    {
        return LimelightHelpers.getTV(cameraName);
    }

    public String getCameraName()
    {
        return cameraName;
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    public void periodic()
    {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // if(poseEstimate != null)
        // {
        //     poseArray[0] = poseEstimate.pose.getX();
        //     poseArray[1] = poseEstimate.pose.getY();
        //     poseArray[2] = poseEstimate.pose.getRotation().getRadians();

        //     poseEntry.set(poseArray);
        // }

        
        cameraPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName).pose;
        if(cameraPose != null)
        {
            cameraEntry.set(cameraPose);
        }
    }

    @Override
    public String toString()
    {
        return "";
    }
}