package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import static frc.robot.Constants.ExampleSubsystem.*;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * LED Subsystem 
 * @author Niyati
 */
public class LEDs
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
    private final List<LEDView> views = new ArrayList<>();


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

    private Runnable actionPattern = null;
    private boolean useActionPattern = false;

    private static Color color = Color.kBlack;

    private LEDPattern base;
    // private LEDPattern progressPattern = LEDPattern.progressMaskLayer(() -> climb.getHeight() / climb.getMaxHieght());

    private Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    public static class LEDView
    {
        private final int startIndex;
        private final int endIndex;
        private final AddressableLEDBufferView bufferView;
        private LEDPattern pattern = LEDPattern.solid(Color.kBlack);
        private boolean isAnimated = false;
        private boolean needsUpdate = true;

        /**
         * Creates the LED view
         * 
         * @param startIndex {@link Integer} The start index of the view
         * @param endIndex {@link Integer} The end index of the view
         */
        private LEDView(int startIndex, int endIndex, AddressableLEDBufferView bufferView)
        {
            this.startIndex = startIndex;
            this.endIndex = endIndex;
            this.bufferView = bufferView;
        }

        /**
         * Sets the pattern of the LED view
         * 
         * @param pattern {@link LEDPattern} The pattern to set to
         * @param isAnimated {@link Boolean} Whether the pattern needs to be updated
         *            constantly
         */
        private void setPattern(LEDPattern pattern, boolean isAnimated)
        {
            if (pattern == null)
            {
                throw new IllegalArgumentException("Pattern cannot be null");
            }

            if (this.pattern.equals(pattern) && this.isAnimated == isAnimated)
            {
                return;
            }

            this.pattern = pattern;
            this.isAnimated = isAnimated;
            this.needsUpdate = true;
        }

        /**
         * Sets the pattern of the LED view to off
         * 
         * @return {@link Command} The command to set the leds in the LED view off
         */
        public Command setOffCommand()
        {
            return Commands.runOnce(() -> setViewColorSolid(80, Color.kBlack));
        }

        /**
         * Sets the pattern of the LED view to a solid color
         * 
         * @param color {@link Color} The color to set the LED view to
         */
        public void setViewColorSolid(int brightness, Color color)
        {
            Objects.requireNonNull(color, "Color cannot be null");
            setPattern(LEDPattern.solid(color).atBrightness(Percent.of(brightness)), false);
        }

        /**
         * Sets the pattern of the LED view to a solid color
         * 
         * @param color {@link Color} The color to set the LED view to
         * @return {@link Command} The command to set the leds in the LED view to a
         *         solid color
         */
        public Command setViewColorSolidCommand(int brightness, Color color)
        {
            return Commands.runOnce(() -> setViewColorSolid(brightness, color));
        }

        /**
         * Sets the pattern of the LED view to a scrolling gradient
         * 
         * @param colors {@link Color} The colors to set the LED view to
         */
        private void setViewColorGradient(int brightness, boolean isAnimated, Color... colors)
        {
            Objects.requireNonNull(colors, "Colors cannot be null");
            setPattern(
                    LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
                            .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(100))
                            .atBrightness(Percent.of(brightness)),
                    isAnimated);
        }

        /**
         * Sets the pattern of the LED view to a scrolling gradient
         * 
         * @param colors {@link Color} The colors to set the LED view to
         * @return {@link Command} The command to set the leds in the LED view to a
         *         scrolling gradient
         */
        public Command setViewColorGradientCommand(int brightness, boolean isAnimated, Color... colors)
        {
            return Commands.runOnce(() -> setViewColorGradient(brightness, isAnimated, colors));
        }

        /**
         * Sets the pattern of the LED view to a scrolling rainbow
         */
        private void setViewColorRainbow(int brightness, boolean isAnimated)
        {
            setPattern(
                    LEDPattern.rainbow(255, 255)
                            .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(100))
                            .atBrightness(Percent.of(brightness)),
                    isAnimated);
        }

        /**
         * Sets the pattern of the LED view to a scrolling rainbow
         * 
         * @return {@link Command} The command to set the leds in the LED view to a
         *         scrolling rainbow
         */
        public Command setViewColorRainbowCommand(int brightness, boolean isAnimated)
        {
            return Commands.runOnce(() -> setViewColorRainbow(brightness, isAnimated));
        }

        /**
         * Modifies the current pattern of the LED view to blink
         * 
         * @param seconds {@link Double} The amount of seconds between each blink
         */
        private void setViewColorBlink(int brightness, Color color, double seconds)
        {
            setViewColorSolid(brightness, color);
            setPattern(this.pattern.blink(Units.Seconds.of(seconds)), true);
        }

        /**
         * Modifies the current pattern of the LED view to blink
         * 
         * @param seconds {@link Double} The amount of seconds between each blink
         * @return {@link Command} The command to set the leds in the LED view to blink
         */
        public Command setViewColorBlinkCommand(int brightness, Color color, double seconds)
        {
            return Commands.runOnce(() -> setViewColorBlink(brightness, color, seconds));
        }

        /**
         * Modifies the current pattern of the LED view to blink
         * 
         * @param offSeconds {@link Double} The amount of seconds to stay off
         * @param onSeconds {@link Double} The amount of seconds to stay on
         */
        private void setViewColorBlink(int brightness, Color color, double offSeconds, double onSeconds)
        {
            setViewColorSolid(brightness, color);
            setPattern(this.pattern.blink(Units.Seconds.of(offSeconds), Units.Seconds.of(onSeconds)).atBrightness(Percent.of(brightness)), true);
        }

        /**
         * Modifies the current pattern of the LED view to blink
         * 
         * @param offSeconds {@link Double} The amount of seconds to stay off
         * @param onSeconds {@link Double} The amount of seconds to stay on
         * @return {@link Command} The command to set the leds in the LED view to blink
         */
        public Command setViewColorBlinkCommand(int brightness, Color color, double offSeconds, double onSeconds)
        {
            return Commands.runOnce(() -> setViewColorBlink(brightness, color, offSeconds, onSeconds));
        }

        /**
         * Modifies the current pattern of the LED view to breathe
         * 
         * @param seconds {@link Double} The amount of seconds between each breathe
         */
        private void setViewColorBreathe(int brightness, Color color, double seconds)
        {
            setViewColorSolid(brightness, color);
            setPattern(this.pattern.breathe(Units.Seconds.of(seconds)), true);
        }

        /**
         * Modifies the current pattern of the LED view to breathe
         * 
         * @param seconds {@link Double} The amount of seconds between each breathe
         * @return {@link Command} The command to set the leds in the LED view to
         *         breathe
         */
        public Command setViewColorBreatheCommand(int brightness, Color color, double seconds)
        {
            return Commands.runOnce(() -> setViewColorBreathe(brightness, color, seconds));
        }
    }

    /** 
     * Creates a new LEDs subsystem. 
     */
    public LEDs()
    {
        // super("LEDs Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        led.setLength(Constants.LEDs.LED_LENGTH);

        configLEDs();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public LEDView createView(int startIndex, int endIndex)
    {
        if (startIndex < 0 || endIndex >= ledBuffer.getLength() || startIndex > endIndex)
        {
            throw new IllegalArgumentException("Invalid LED view bounds");
        }

        for (LEDView existing : views)
        {
            if (existing.startIndex == startIndex && existing.endIndex == endIndex)
            {
                return existing;
            }

            if (startIndex <= existing.endIndex && endIndex >= existing.startIndex)
            {
                throw new IllegalArgumentException(String.format("View [%s, %s] overlaps with existing view [%s, %s]",
                        startIndex, endIndex, existing.startIndex, existing.endIndex));
            }
        }

        AddressableLEDBufferView bufferView = ledBuffer.createView(startIndex, endIndex);
        LEDView view = new LEDView(startIndex, endIndex, bufferView);
        views.add(view);

        return view;
    }

    public void deleteView(LEDView view)
    {
        view.pattern = LEDPattern.solid(Color.kBlack);
        view.pattern.applyTo(view.bufferView);
        led.setData(ledBuffer);

        views.remove(view);
    }

    public void deleteAllViews()
    {
        views.clear();
    }

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
        LEDs.color = color;
    }

    /**
     * This sets the LEDs to blink
     * @param color The LED colors
     * @param brightness The LED brightness
     */
    private void setColorBlink(int brightness, Color color)
    {
        // base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
        base = LEDPattern.solid(color).atBrightness(Percent.of(brightness));
        blink = base.breathe(Units.Seconds.of(1));
        blink.applyTo(ledBuffer);
        LEDs.color = color;
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
        LEDs.color = colors[0];
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
        LEDs.color = colors[0];
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
        LEDs.color = colors[0];
    }

    /**
     * This sets the LEDs to rainbow
     */
    private void setColorRainbow()
    {
        rainbow.applyTo(ledBuffer);
        LEDs.color = Color.kBlack;
    }

    private void setMovingRainbow()
    {
        base = LEDPattern.rainbow(255, 255);
        mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(200));
        movingRainbow = base.mask(mask);
        movingRainbow.applyTo(ledBuffer);
        LEDs.color = Color.kBlack;
    }

    private void off()
    {
        off.applyTo(ledBuffer);
        led.setData(ledBuffer);
        LEDs.color = Color.kBlack;
    }

    // COMMANDS

    public Command setColorSolidCommand(int brightness, Color color)
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = false;
            actionPattern = null;
            setColorSolid(brightness, color);
            System.out.println("Setting solid color");
        }
        ).withName("Set LED Solid");
    }

    public Command setColorGradientCommand(int brightness, Color ...colors)
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = false;
            setColorGradient(brightness, colors);
        }
        ).withName("Set LED Gradient");
    }

    public Command setColorBlinkCommand(int brightness, Color color)
    {
        return Commands.runOnce(() -> 
            {
                useActionPattern = true; 
                actionPattern = () -> setColorBlink(brightness, color);
            }
            ).withName("Set LED Blink");
    }

    public Command setColorBreatheCommand(int brightness, Color ...colors)
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = true; 
            actionPattern = () -> setColorBreathe(brightness, colors);
        }
        ).withName("Set LED Breathe");
    }

    public Command setColorProgressBarCommand(int brightness, Color ...colors)
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = false;
            setColorProgressBar(brightness, colors);
        }
        ).withName("Set LED Progress Bar");
    }

    public Command setColorRainbowCommand()
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = false;
            setColorRainbow();
        }
        ).withName("Set LED Rainbow");
    }

    public Command setMovingRainbowCommand()
    {
        return Commands.runOnce(() -> 
            {
                useActionPattern = true; 
                actionPattern = () -> setMovingRainbow();
            }
            ).withName("Set LED Moving Rainbow");
    }

    public Command offCommand()
    {
        return Commands.runOnce(() -> 
        {
            useActionPattern = false;
            off();
        }
        ).withName("Turn off LEDs");
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    // @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        if (useActionPattern)
        {
            // System.out.println("LED animation running");
            actionPattern.run();
        }

        // boolean needsUpdate = false;

        for (LEDView view : views)
        {
            if (view.isAnimated || view.needsUpdate)
            {
                view.pattern.applyTo(view.bufferView);
                view.needsUpdate = false;
                // needsUpdate = true;
            }
        }

        // if (needsUpdate)
        // {
        //     led.setData(ledBuffer);
        // }

        led.setData(ledBuffer);
    }

    @Override
    public String toString()
    {
        return "";
    }
}
