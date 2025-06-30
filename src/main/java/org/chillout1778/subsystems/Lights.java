package org.chillout1778.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Robot;

public class Lights extends SubsystemBase {
    private static Lights instance;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private static final int LENGTH = 86; // probs should be a constant
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuff;

    // LED segments
    private AddressableLEDBufferView rightSegment;
    private AddressableLEDBufferView crossSegment;
    private AddressableLEDBufferView leftSegment;
    private LEDPattern blackPattern = LEDPattern.solid(Color.kBlack);
    private Timer lightsTimer = new Timer();

    // Additional LED patterns and animation variables
    private LEDPattern blinkyPattern = LEDPattern.solid(LedColors.GoodGreen.color).blink(Units.Seconds.of(0.15));
    private double progressBarAnimation = 0.0;
    private LEDPattern disabledRainbow = LEDPattern.rainbow(255, 255)
            .scrollAtRelativeSpeed(Units.Hertz.of(0.5));

    private Lights() {
        leds = new AddressableLED(3);
        leds.setColorOrder(AddressableLED.ColorOrder.kRGB);
        ledBuff = new AddressableLEDBuffer(LENGTH);

        rightSegment = ledBuff.createView(0, 32); // right segment
        crossSegment = ledBuff.createView(33, 52); // cross segment
        leftSegment = ledBuff.createView(53, 85).reversed(); // left segment

        ledBuff.createView(34, 48);
        ledBuff.createView(49, 53).reversed();

        lightsTimer.reset();
        lightsTimer.start();

        leds.setLength(LENGTH);
        leds.start();
    }

    @Override
    public void periodic() {
        RobotStatus status = getDisabledStatus();
        if (Robot.wasEnabled) {
            if (RobotController.isBrownedOut()) {
                writeLedColor(LedColors.PoopBrown);
            } else if (!status.CANHealthy) {
                writeLedColor(LedColors.BadRed);
            } else if (Swerve.getInstance().isAligned) {
                blinkGreen();
            } else if (Arm.getInstance().hasObject) {
                writeLedColor(LedColors.PureWhite);
            } else if (Intake.getInstance().hasCoral()) {
                writeLedColor(LedColors.LightBlue);
            } else {
                writeLedColor(LedColors.DarkBlue);
            }
            if (Arm.getInstance().isArmStuck) {
                setCrossbarColor(LedColors.BadRed);
            }
        } else {
            writeDisabledStatus(status);
            disabledAnimations();
        }

        leds.setData(ledBuff);
    }

    public void writeLedColor(LedColors c) {
        c.solidPattern.applyTo(ledBuff);
    }

    public static class RobotStatus {
        public final boolean CANHealthy;
        public final boolean armCorrectOrientation;
        public final double batteryVoltage;
        public final boolean camerasConnected;
        public final boolean brakeMode;
        public final boolean intakeHasCoral;

        public RobotStatus(boolean CANHealthy, boolean armCorrectOrientation, double batteryVoltage,
                boolean camerasConnected, boolean brakeMode, boolean intakeHasCoral) {
            this.CANHealthy = CANHealthy;
            this.armCorrectOrientation = armCorrectOrientation;
            this.batteryVoltage = batteryVoltage;
            this.camerasConnected = camerasConnected;
            this.brakeMode = brakeMode;
            this.intakeHasCoral = intakeHasCoral;
        }
    }

    public enum LedColors {
        GoodGreen(new Color(0, 255, 0)),
        BadRed(new Color(255, 0, 0)),
        WarningYellow(new Color(255, 255, 0)),
        PureWhite(new Color(255, 255, 255)),
        LightBlue(new Color(0, 175, 255)),
        DarkBlue(new Color(0, 0, 255)),
        PoopBrown(new Color(160, 82, 45)),
        TotalBlack(Color.kBlack);

        public final Color color;
        public final LEDPattern solidPattern;

        LedColors(Color color) {
            this.color = color;
            this.solidPattern = LEDPattern.solid(color);
        }
    }

    private RobotStatus getDisabledStatus() {
        // Simplified status check - using safe defaults for now
        return new RobotStatus(
                true, // CAN healthy - simplified
                Math.abs(0.5) == 0.5, // arm correct orientation - simplified
                12.7, // battery voltage
                true, // cameras connected - simplified
                true, // brake mode - simplified
                Intake.getInstance().hasCoral());
    }

    private Color boolColor(boolean b) {
        return b ? LedColors.DarkBlue.color : LedColors.BadRed.color;
    }

    private void blinkGreen() {
        blinkyPattern.applyTo(ledBuff);
    }

    private void setCrossbarColor(LedColors color) {
        color.solidPattern.applyTo(crossSegment);
    }

    private void writeDisabledStatus(RobotStatus status) {
        blackPattern.applyTo(ledBuff);
        LedColors.DarkBlue.solidPattern.applyTo(crossSegment);

        ledBuff.setLED(48, boolColor(status.camerasConnected));
        ledBuff.setLED(47, boolColor(status.camerasConnected));

        ledBuff.setLED(46, boolColor(status.CANHealthy));
        ledBuff.setLED(45, boolColor(status.CANHealthy));

        ledBuff.setLED(44, boolColor(status.armCorrectOrientation));
        ledBuff.setLED(43, boolColor(status.armCorrectOrientation));

        ledBuff.setLED(42, boolColor(status.brakeMode));
        ledBuff.setLED(41, boolColor(status.brakeMode));

        ledBuff.setLED(40, boolColor(!status.intakeHasCoral));
        ledBuff.setLED(39, boolColor(!status.intakeHasCoral));
    }

    private void disabledAnimations() {
        if (Robot.wasEnabledThenDisabled && lightsTimer.get() <= 5.0) {
            disabledRainbow.applyTo(rightSegment);
            disabledRainbow.applyTo(leftSegment);
            lightsTimer.restart();
        } else if (Robot.wasEnabledThenDisabled && lightsTimer.get() > 5.0) {
            Robot.wasEnabledThenDisabled = false;
            lightsTimer.restart();
            // first two seconds after robot code boots up, complete a blue progressBar
        } else if (lightsTimer.get() <= 2.0) {
            progressBarAnimation += 0.02;

            if (progressBarAnimation >= 1.0) {
                progressBarAnimation = 0.0;
            }

            LEDPattern progressBarPattern = LEDPattern.solid(LedColors.LightBlue.color)
                    .mask(LEDPattern.progressMaskLayer(() -> progressBarAnimation));

            progressBarPattern.applyTo(rightSegment);
            progressBarPattern.applyTo(leftSegment);
            // then recursively, every 10 seconds run a light across the bar
        } else if (lightsTimer.get() % 10.0 <= 1.0) {
            nuclearRats(2.0, LedColors.TotalBlack, LedColors.LightBlue);
        }
    }

    public void nuclearRats(double frequency, LedColors baseColor, LedColors overlayColor) {
        LEDPattern overlayStepsPattern = LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous,
                baseColor.color,
                overlayColor.color);
        LEDPattern overlayStepsFinal = overlayStepsPattern.scrollAtRelativeSpeed(Units.Hertz.of(frequency));

        LEDPattern overlayCenterMask = LEDPattern.solid(baseColor.color);
        LEDPattern overlayCenterFlashed = overlayCenterMask.blink(
                Units.Seconds.of(1.0 / frequency),
                Units.Seconds.of(1.0 / frequency));
        LEDPattern overlayFinalPattern = overlayCenterMask.overlayOn(overlayCenterFlashed);

        overlayStepsFinal.applyTo(leftSegment);
        overlayStepsFinal.applyTo(rightSegment);
        if (baseColor != LedColors.TotalBlack)
            overlayFinalPattern.applyTo(crossSegment);
    }
}
