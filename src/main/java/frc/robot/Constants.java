// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    public static final String ADVANTAGE_SCOPE_TABLE_NAME = "ASTable";

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    /** 
     * These are the names of the CAN bus set on the roboRIO and CANivore
     */
    public static class CANbus
    {
        public static final String CANIVORE = "CANivore";
        public static final String ROBORIO  = "rio";
    }

    /**
     * This class contains the constants used for the ExampleSubsystem
     */
    public static class ExampleSubsystem
    {
        public static final String MOTOR_CAN_BUS = CANbus.ROBORIO;

        public static final int MOTOR1 = 1;
        public static final int MOTOR2 = 2;
    }

    public static class Agitator
    {
        public static final int MOTOR                   = 1;

        public static final String MOTOR_CAN_BUS        = CANbus.ROBORIO;

    }

    public static class Climb
    {
        public static final int LEADMOTOR          = 4237;
        public static final int FOLLOWMOTOR        = 4237;

        public static final String MOTOR_CAN_BUS        = CANbus.ROBORIO;    
    }

    public static class Accelerator
    {
        public static final int MOTOR                   = 3;

        public static final String MOTOR_CAN_BUS        = CANbus.ROBORIO;
    }

    public static class Flywheel
    {
        public static final int LEADMOTOR               = 1;
        public static final int FOLLOWMOTOR             = 2;

        public static final String MOTOR_CAN_BUS        = CANbus.ROBORIO;
    }
    
    public static class Indexer
    {
        public static final int MOTOR                   = 2;

        public static final String MOTOR_CAN_BUS        = CANbus.ROBORIO;
    }

    public static class Intake
    {
        public static final int INTAKEPIVOTMOTOR            = 4;
        public static final int INTAKEROLLERLEADER          = 3;
        public static final int INTAKEROLLERFOLLOWER        = 12;

        public static final String MOTOR_CAN_BUS            = CANbus.ROBORIO;
    }   

    /**
     * This class contains the port numbers of the controllers
     */
    public static class Controllers
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /**
     * This class contains the names of the Network Tables for logging data
     */
    public static class NetworkTableLance
    {
        public static final String TEAM_TABLE = "TeamLance";
        public static final String ADVANTAGE_SCOPE_TABLE = "ASTable";
    }

    public static class LEDs
    {
        public static final int LED_LENGTH = 60;
        public static final int LED_PORT = 9;
    }

}
