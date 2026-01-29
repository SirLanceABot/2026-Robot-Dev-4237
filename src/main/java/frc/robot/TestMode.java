package frc.robot;

import java.lang.invoke.MethodHandles;
import frc.robot.tests.Test;

// *** IMPORTANT - PLEASE READ ***
// 1. Put your test code in your own frc.robot.tests.[yourname]Test.java file
// 2. Uncomment your IMPORT statement below
// 3. Uncomment your INITIALIZATION statement below
// 4. Use the RobotContainer class to construct needed objects
// 5. Test your code
// 6. Undo the changes to this file when finished with testing


// *** IMPORT statements ***
// Uncomment one of these statements

// import frc.robot.tests.BradyWTest;
// import frc.robot.tests.GretaHTest;
// import frc.robot.tests.LoganBTest;
// import frc.robot.tests.MatthewFTest;
// import frc.robot.tests.NiyatiPTest;
import frc.robot.tests.RobbieFTest;
// import frc.robot.tests.RobbieJTest;
// import frc.robot.tests.JWoodTest;
// import frc.robot.tests.RickC137Test;
// import frc.robot.tests.MaxMTest;


public class TestMode
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES ***
    private Test myTest = null;
    

    public TestMode(RobotContainer robotContainer)
    {
        // *** INITIALIZATION statements ***
        // Uncomment one of these statements

        // myTest = new BradyWTest(robotContainer);
        // myTest = new GretaHTest(robotContainer);
        // myTest = new LoganBTest(robotContainer);
        // myTest = new MatthewFTest(robotContainer);
        // myTest = new NiyatiPTest(robotContainer);
        myTest = new RobbieFTest(robotContainer);
        // myTest = new RobbieJTest(robotContainer);
        // myTest = new JWoodTest(robotContainer);
        // myTest = new RickC137Test(robotContainer);
        // myTest = new MaxMTest(robotContainer);
    }

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
        if(myTest != null)
            myTest.init();
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        if(myTest != null)
            myTest.periodic();
    }

    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {
        if(myTest != null)
            myTest.exit();

        // Set the Test object to null so that garbage collection will remove the object.
        myTest = null;
    }    
}
