package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import static frc.robot.Constants.ExampleSubsystem.*;

import java.lang.invoke.MethodHandles;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * LED Subsystem 
 * @author Niyati
 */
public class LEDs extends SubsystemBase
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

    public enum ColorPattern
    {
        kSolid,
        kBlink,
        kGradient,
        kBreathe,
        kProgressBar,
        kRainbow,
        kOff
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private final AddressableLED led = new AddressableLED(Constants.LEDs.LED_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);

    // All Patterns
    private LEDPattern solid;
    private LEDPattern blink;
    private LEDPattern gradient;
    private LEDPattern breathe;
    private LEDPattern progressBar;
    private LEDPattern movingRainbow;
    private LEDPattern mask;
    private LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    private LEDPattern off = LEDPattern.solid(Color.kBlack);

    private static Color color = Color.kBlack;

    private LEDPattern base;
    // private LEDPattern progressPattern = LEDPattern.progressMaskLayer(() -> climb.getHeight() / climb.getMaxHieght());

    private Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new LEDs subsystem. 
     */
    public LEDs()
    {
        super("LEDs Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        led.setLength(Constants.LEDs.LED_LENGTH);

        configLEDs();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configLEDs()
    {
        led.start();
    }

    public static Color getColor()
    {
        return color;
    }

    /**
     * This sets the LEDs to a solid color
     * @param color The LED color
     * @param brightness The LED brightness
     */
    private void setColorSolid(int brightness, Color color)
    {
        solid = LEDPattern.solid(color).atBrightness(Percent.of(brightness));
        solid.applyTo(ledBuffer);
        this.color = color;
    }

    /**
     * This sets the LEDs to blink
     * @param color The LED colors
     * @param brightness The LED brightness
     */
    private void setColorBlink(Color... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        blink = base.breathe(Units.Seconds.of(0.5));
        blink.applyTo(ledBuffer);
        this.color = colors[0];
    }
    
    /**
     * This sets the LEDs to a gradient
     * @param color The LED colors
     * @param brightness The LED brightness
     */
    private void setColorGradient(int brightness, Color... colors)
    {
        gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
        gradient.applyTo(ledBuffer);
        this.color = colors[0];
    }

    /*C
     * This sets the LEDs to breathe pattern
     * @param color The LED colors
     * @param brightness The LED brightness
     */
    private void setColorBreathe(int brightness, Color... colors)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        breathe = base.breathe(Units.Seconds.of(2));
        breathe.applyTo(ledBuffer);
        this.color = colors[0];
    }

    /**
     * This sets the LEDs to a progress bar
     * @param color The LED colors
     * @param brightness The LED brightness
     */
    private void setColorProgressBar(int brightness, Color... colors)
    {
        // base = LEDPattern.progressMaskLayer(() -> Climb.getPosition() / Climb.climbPosition.kL1);
        progressBar.applyTo(ledBuffer);
        this.color = colors[0];
    }

    /**
     * This sets the LEDs to rainbow
     */
    private void setColorRainbow()
    {
        rainbow.applyTo(ledBuffer);
        this.color = Color.kBlack;
    }

    private void setMovingRainbow()
    {
        base = LEDPattern.rainbow(255, 255);
        mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(200));
        movingRainbow = base.mask(mask);
        movingRainbow.applyTo(ledBuffer);
        this.color = Color.kBlack;
    }

    private void off()
    {
        off.applyTo(ledBuffer);
        led.setData(ledBuffer);
        this.color = Color.kBlack;
    }

    // COMMANDS

    public Command setColorSolidCommand(int brightness, Color color)
    {
        return runOnce(() -> setColorSolid(brightness, color)).withName("Set LED Solid");
    }

    public Command setColorGradientCommand(int brightness, Color ...colors)
    {
        return runOnce(() -> setColorGradient(brightness, colors)).withName("Set LED Gradient");
    }

    public Command setColorBlinkCommand(Color ...colors)
    {
        return runOnce(() -> setColorBlink(colors)).withName("Set LED Blink");
    }

    public Command setColorBreatheCommand(int brightness, Color ...colors)
    {
        return runOnce(() -> setColorBreathe(brightness, colors)).withName("Set LED Breathe");
    }

    public Command setColorProgressBarCommand(int brightness, Color ...colors)
    {
        return runOnce(() -> setColorProgressBar(brightness, colors)).withName("Set LED Progress Bar");
    }

    public Command setColorRainbowCommand()
    {
        return runOnce(() -> setColorRainbow()).withName("Set LED Rainbow");
    }

    public Command setMovingRainbowCommand()
    {
        return run(() -> setMovingRainbow()).withName("Set LED Moving Rainbow");
    }

    public Command offCommand()
    {
        return runOnce(() -> off()).withName("Turn off LEDs");
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        led.setData(ledBuffer);
    }

    @Override
    public String toString()
    {
        return "";
    }
}
