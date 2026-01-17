// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import frc.robot.subsystems.ExampleSubsystem;

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

    // *** STATTIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // Select the robot components to use
    private boolean useFullRobot = false;
    private boolean useExampleSubsystem = false;

    // Robot components
    private ExampleSubsystem exampleSubsystem = null;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        // Instantiate ONLY the components selected above
        if(useFullRobot || useExampleSubsystem)
            exampleSubsystem = new ExampleSubsystem();
    }

    public ExampleSubsystem getExampleSubsystem()
    {
        return exampleSubsystem;
    }

}
