// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Accelerator;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** 
 * Commands 
 */
public class IntakeAndScoreCommand extends Command 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS AND INSTANCE VARIABLES ***
    private final Intake intake; 
    private final Agitator agitator;
    private final Indexer indexer;
    private final Accelerator accelerator;
    private final Flywheel flywheel;


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeAndScoreCommand(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel) 
    {
        this.intake = intake; 
        this.agitator = agitator;
        this.indexer = indexer;
        this.accelerator = accelerator;
        this.flywheel = flywheel;
        // Use addRequirements() here to declare subsystem dependencies.
        if(this.intake != null && this.agitator != null && this.indexer != null  && this.accelerator != null  && this.flywheel != null )
        {
            addRequirements(this.intake);
            addRequirements(this.agitator);
            addRequirements(this.indexer);
            addRequirements(this.accelerator);
            addRequirements(this.flywheel);
        }
    }

    public static Command IntakeAndScoreCommand(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.pickupFuelCommand()),
                (agitator.forwardCommand()),
                (indexer.onCommand()),
                (accelerator.feedToShooterCommand(()-> 0.25)),
                (flywheel.shootCommand(() -> 10.0)));
        }
        return null; 
    } 

    public static Command IntakeAndScoreCommandTheSequal(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel( (accelerator.feedToShooterCommand(() -> 0.25)),
                    (flywheel.shootCommand(() -> 75.7))).until(flywheel.isAtSetSpeed(100, 5))
                    .andThen
                    // .commands.parallel(
                (intake.pickupFuelCommand()).andThen
                (agitator.forwardCommand()).andThen
                (indexer.onCommand());

        }
        return null; 
    } 


    public static Command StopIntakeAndScoreCommand(Intake intake, Agitator agitator, Indexer indexer, Accelerator accelerator, Flywheel flywheel)
    {

        
        if(intake != null && agitator != null && indexer != null  && accelerator != null  && flywheel != null )
        {
            return  Commands.parallel(
                (intake.stopCommand()),
                (agitator.stopCommand()),
                (indexer.stopCommand()),
                (accelerator.stopCommand()),
                (flywheel.stopCommand()));
        }
        return null; 
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return false;
    }
    
    @Override
    public boolean runsWhenDisabled()
    {
        return false;
    }

    @Override
    public String toString()
    {
        String str = this.getClass().getSimpleName();
        return String.format("Command: %s( )", str);
    }
}
