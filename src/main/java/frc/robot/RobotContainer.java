// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
    // This string gets the full name of the class including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // Select the robot components to use
    private boolean useFullRobot = false;
    private boolean useExampleSubsystem = false;
    private boolean useAgitator = false;
    private boolean useIndexer = false;
    private boolean useFlywheel = false;
    private boolean useIntake = false;
    private boolean useAccelerator = false;
    private boolean useClimb = false;

    // Robot components
    private ExampleSubsystem exampleSubsystem = null;
    private Agitator agitator = null;
    private Indexer indexer = null;
    private Flywheel flywheel = null;
    private Intake intake = null;
    private Accelerator accelerator;
    private Climb climb = null;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        // Instantiate ONLY the components selected above
        if(useFullRobot || useExampleSubsystem)
            exampleSubsystem = new ExampleSubsystem();

        if(useFullRobot || useAgitator)
            agitator = new Agitator(); 

        if(useFullRobot || useIndexer)
            indexer = new Indexer();

        if(useFullRobot || useFlywheel)
            flywheel = new Flywheel();
        
        if(useFullRobot || useIntake)
            intake = new Intake();

        if(useFullRobot || useAccelerator)
            accelerator = new Accelerator();

        if(useFullRobot || useClimb)
            climb = new Climb();
    }

    public ExampleSubsystem getExampleSubsystem()
    {
        return exampleSubsystem;
    }

    public Agitator getAgitator()
    {
        return agitator;
    }

    public Indexer getIndexer()
    {
        return indexer;
    }

    public Flywheel getFlywheel()
    {
        return flywheel;
    }

    public Intake getIntake()
    {
        return intake;
    }

    public Accelerator getAccelerator()
    {
        return accelerator;
    }

    public Climb getClimb()
    {
        return climb;
    }
}
