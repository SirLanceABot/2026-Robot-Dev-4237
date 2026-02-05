package frc.robot.sensors;

import java.io.OutputStream;
import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class HopperCamera
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
    int width = 640;
    int height = 480;
    Mat mask = new Mat(height, width, CvType.CV_8UC1, new Scalar(0));
    private final GripPipeline pipeline = new GripPipeline();
    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a HopperCamera. 
     */
    public HopperCamera()
    {   


        System.out.println("  Constructor Started:  " + fullClassName);

            Thread m_visionThread = new Thread(
                            () -> {
                            // Get the UsbCamera from CameraServer
                            UsbCamera camera = CameraServer.startAutomaticCapture();
                            // Set the resolution
                            camera.setResolution(width, height);
                            Imgproc.rectangle(mask, new Point(width,0), new Point(0,height - 20 ), new Scalar(255), -1);



                            // Get a CvSink. This will capture Mats from the camera
                            CvSink cvSink = CameraServer.getVideo();
                            // Setup a CvSource. This will send images back to the Dashboard
                            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
            
                            // Mats are very memory expensive. Lets reuse this Mat.
                            Mat mat = new Mat();
            
                            // This cannot be 'true'. The program will never exit if it is. This
                            // lets the robot stop this thread when restarting robot code or
                            // deploying.
                            while (!Thread.interrupted()) {
                                // Tell the CvSink to grab a frame from the camera and put it
                                // in the source mat.  If there is an error notify the output.
                                if (cvSink.grabFrame(mat) == 0) {
                                // Send the output the error.
                                outputStream.notifyError(cvSink.getError());
                                // skip the rest of the current iteration
                                continue;
                                }
                                
                                pipeline.process(mat,mask);

                                // Give the output stream a new image to display

                                outputStream.putFrame(mat);
                            }   
            
                            });
                        
        m_visionThread.setDaemon(true);
        m_visionThread.start();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    public Mat FilterToYellowBGR(Mat src)
    {
        Mat dst = new Mat();
        Scalar low = new Scalar(0, 150, 150);   // B, G, R
        Scalar high = new Scalar(100, 255, 255);
        Core.inRange(src, low, high, dst);
        // HighGui.imshow("Original", src);
        // HighGui.imshow("Filtered", dst);
        // HighGui.waitKey();


        return dst;
    }


    public Mat FilterToYellowHSV(Mat src)
    {
        Mat dst = new Mat();
        Imgproc.cvtColor(src, src,40); //           COLOR_BGR2HSV = 40,
        Scalar high = new Scalar(60, 100, 100);   // Hue, Saturation, Value
        Scalar low = new Scalar(60, 50, 34);
        Core.inRange(src, low, high, dst);
        // HighGui.imshow("Original", src);
        // HighGui.imshow("Filtered", dst);
        // HighGui.waitKey();


        return dst;
    }





    /**
     * Returns the value of the sensor
    * @return The value of periodData.sensorValue
    */


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "";
    }
}