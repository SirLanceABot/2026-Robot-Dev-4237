package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;


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
    int width = 640/4;
    int height = 480/4;
    Timer timer = new Timer();
    private volatile boolean isHopperFull = false;
    
    private final GripPipeline pipeline = new GripPipeline();
    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a HopperCamera. 
     */
    public HopperCamera()
    {   
        

        System.out.println("  Constructor Started:  " + fullClassName);

        timer.reset();

        Thread m_visionThread = new Thread(
            () -> {
             // Get the UsbCamera from CameraServer
                UsbCamera camera = CameraServer.startAutomaticCapture();
                // Set the resolution
                camera.setResolution(width, height);
                // Mat mask = new Mat(height, width, CvType.CV_8UC1, new Scalar(0));
                // Imgproc.rectangle(mask, new Point(0,0), new Point(width,100 ), new Scalar(255), -1);



                // Get a CvSink. This will capture Mats from the camera
                CvSink cvSink = CameraServer.getVideo();
                // Setup a CvSource. This will send images back to the Dashboard
                CvSource outputStream = CameraServer.putVideo("outputStream", 640, 480);
            
                // Mats are very memory expensive. Lets reuse this Mat.
                Mat mat = new Mat();
                Scalar avg;
                //Needs to be tuned 
                double threshold = 1;

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
                                
                pipeline.process(mat);
// Imgproc.rectangle(
//                     mat, new Point(0, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(pipeline.cvRectangleOutput());
           
                avg = Core.mean(pipeline.cvRectangleOutput());
                if(avg.val[0] >= threshold)
                {
                    if (timer.get() == 0)
                    {
                        timer.start();
                    }
                    else if(timer.get() >= 0.5)
                    {
                        // System.err.println("YELLOW");
                        isHopperFull = true;
                    }
                }
                else
                {
                    timer.reset();
                    // System.err.println("NOT YELLOW");
                    isHopperFull = false;
                }
                }   
            
                });
                        
        m_visionThread.setDaemon(true);
        m_visionThread.start();

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    public BooleanSupplier isHoppperFullSupplier()
    {
        return (() -> isHopperFull);
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here
    @Override
    public String toString()
    {
        return "";
    }
}