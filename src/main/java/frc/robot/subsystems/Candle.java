package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

/**
 * An LED strip that can be used to display animations or colors
 **/
public class Candle extends SubsystemBase {
    /**
     * A CANdle instance that controls the LEDs
     **/
    private final CANdle candle = new CANdle(Constants.Ports.CANDLE, Constants.Ports.CTRE_CANBUS);

    /**
     * The currently selected animation
     **/
    private Animation toAnimate;
    /**
     * A boolean that represents if an LED action that isn't an animation is occurring
     **/
    private boolean specialAnimation;

    /**
     * An enum containing all possible animation types
     **/
    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Error,
        Twinkle,
        TwinkleOff,
        Empty
    }

    public static Candle instance;

    private Candle() {
        this.setAnimation(AnimationTypes.Empty);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = CANdle.LEDStripType.GRB;
        config.brightnessScalar = Constants.CANDLE_BRIGHTNESS;
        this.candle.configAllSettings(config);
    }

    public static Candle getInstance() {
        return Candle.instance == null ? Candle.instance = new Candle() : Candle.instance;
    }

    /**
     * Called periodically to follow the requested animation
     **/
    @Override
    public void periodic() {
        if (this.toAnimate != null && !this.specialAnimation) {
            this.candle.animate(this.toAnimate);
//    } else if (specialAnimation) {
            // do nothing the LED color is set to a color or turned off
        }
    }

    /**
     * Sets the color of the Candle LEDs
     * @param r The red value of the requested color
     * @param g The green value of the requested color
     * @param b The blue value of the requested color
     **/
    public Command setColor(int r, int g, int b) {
        this.setAnimation(AnimationTypes.Empty);
        return this.runOnce(() -> {
            this.specialAnimation = true;
            this.candle.setLEDs(r, g, b);
        });

    }

    /**
     * Sets the color of the Candle for a limited duration.
     * @param r The red value of the requested color
     * @param g The green value of the requested color
     * @param b The blue value of the requested color
     * @param duration How long to have the requested color in seconds
     **/
    public Command setColor(int r, int g, int b, double duration) {
        return this.setColor(r, g, b).withTimeout(duration).andThen(this.turnOff());
    }

    /**
     * Turns off the LEDs
     *
     * @return
     */
    public Command turnOff() {
        return this.runOnce(()-> {
            this.specialAnimation = true;
            this.candle.setLEDs(0, 0, 0);
        });
    }

    public Command flash(int r, int g, int b, int flashDuration, int duration){
        return this.run(()->{
            this.setColor(r, g, b);
            new WaitCommand(flashDuration);
        }).repeatedly().withTimeout(duration);
    }

    /**
     * Changes the animation for a duration of time
     * @param animation The requested animation
     * @param duration How long the requested animation should be set for in seconds
     **/
    public void setAnimation(AnimationTypes animation, double duration) {
        this.specialAnimation = false;
        switch (animation) {
            default:
            case Empty:
                this.toAnimate = null;
                break;
            case ColorFlow:
                this.toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION, ColorFlowAnimation.Direction.Forward);
                break;
            case Fire:
//                this.toAnimate = new FireAnimation(0.5, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION, 0.8, 0.5);
                break;
            case Larson:  // Mr. Larson reference???
                this.toAnimate = new LarsonAnimation(0, 255, 46);
                break;
            case Rainbow:
                this.toAnimate = new RainbowAnimation(1, 0.7, Constants.CANDLE_LEDS_PER_ANIMATION);
                break;
            case RgbFade:
                this.toAnimate = new RgbFadeAnimation(0.7, 0.4, Constants.CANDLE_LEDS_PER_ANIMATION);
                break;
            case SingleFade:
                this.toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANDLE_LEDS_PER_ANIMATION);
                break;
            case Strobe:
                this.toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.5, Constants.CANDLE_LEDS_PER_ANIMATION);
                break;
            case Error:
                this.toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.01, Constants.CANDLE_LEDS_PER_ANIMATION);
            case Twinkle:
                this.toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, Constants.CANDLE_LEDS_PER_ANIMATION, TwinkleAnimation.TwinklePercent.Percent42);
                break;
            case TwinkleOff:
                this.toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, Constants.CANDLE_LEDS_PER_ANIMATION, TwinkleOffAnimation.TwinkleOffPercent.Percent76);
                break;
        }
        if (duration > 0) {
            CommandScheduler.getInstance().schedule((new WaitCommand(duration)).andThen(() -> this.setAnimation(AnimationTypes.Empty)));
        }
    }

    public Command setAnimation(AnimationTypes animation) {
       return this.runOnce(() -> {
           this.setAnimation(animation, 0.0);
       });
    }
}