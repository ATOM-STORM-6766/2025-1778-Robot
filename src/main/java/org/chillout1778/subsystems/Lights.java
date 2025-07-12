package org.chillout1778.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Robot;
import org.chillout1778.Constants.DioIds;

public class Lights extends SubsystemBase {
  private static Lights instance;

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }
    return instance;
  }

  // right - 34 leds, left - 33, cross - 20
  private static final int LENGTH = 86; // probs should be a constant
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuff;

  // LED segments
  private final AddressableLEDBufferView rightSegment;
  private final AddressableLEDBufferView crossSegment;
  private final AddressableLEDBufferView leftSegment;

  private final LEDPattern blackPattern = LEDPattern.solid(Color.kBlack);
  private final Timer lightsTimer = new Timer();
  private double progressBarAnimation = 0.0;
  private final LEDPattern disabledRainbow =
      LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.5, Units.Hertz));
  private final LEDPattern blinkyPattern =
      LEDPattern.solid(LedColors.GoodGreen.color).blink(Units.Seconds.of(0.15));

  private Lights() {
    leds = new AddressableLED(DioIds.ADDRESSABLE_LED);
    leds.setColorOrder(AddressableLED.ColorOrder.kRGB);
    ledBuff = new AddressableLEDBuffer(LENGTH);

    rightSegment = ledBuff.createView(0, 32); // right segment
    crossSegment = ledBuff.createView(33, 52); // cross segment
    leftSegment = ledBuff.createView(53, 85).reversed(); // left segment
    ledBuff.createView(34, 48); // battery charge progress bar
    ledBuff.createView(49, 53).reversed();

    lightsTimer.reset();
    lightsTimer.start();

    leds.setLength(LENGTH);
    leds.start();
  }

  @Override
  public void periodic() {
    RobotStatus status = getDisabledStatus();
    if (Robot.getInstance().isEnabled()) {
      if (RobotController.isBrownedOut()) {
        writeLedColor(LedColors.PoopBrown);
      } else if (!status.CANHealthy) {
        writeLedColor(LedColors.BadRed);
      } else if (SwerveNext.getInstance().getIsAligned()) {
        blinkGreen();
      } else if (Arm.getInstance().getHasObject()) {
        writeLedColor(LedColors.PureWhite);
      } else if (Intake.getInstance().hasCoral()) {
        writeLedColor(LedColors.LightBlue);
      } else {
        writeLedColor(LedColors.DarkBlue);
      }
      if (Arm.getInstance().getIsArmStuck()) {
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

    public RobotStatus(
        boolean CANHealthy,
        boolean armCorrectOrientation,
        double batteryVoltage,
        boolean camerasConnected,
        boolean brakeMode,
        boolean intakeHasCoral) {
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
    return new RobotStatus(
        Arm.getInstance().getArmPivotMotor().isConnected()
            && Arm.getInstance().getRollerMotor().isConnected(),
        Math.abs(Arm.getInstance().getCloseClampedPosition()) == 0.5,
        12.7,
        Vision.getInstance().allConnected(),
        !Robot.getInstance().getWasCoastModeEnabled(),
        Intake.getInstance().hasCoral());
  }

  private Color boolColor(boolean b) {
    return b ? LedColors.DarkBlue.color : LedColors.BadRed.color;
  }

  public void writeDisabledStatus(RobotStatus state) {
    blackPattern.applyTo(ledBuff);
    LedColors.DarkBlue.solidPattern.applyTo(crossSegment);

    ledBuff.setLED(48, boolColor(state.camerasConnected));
    ledBuff.setLED(47, boolColor(state.camerasConnected));

    ledBuff.setLED(46, boolColor(state.CANHealthy));
    ledBuff.setLED(45, boolColor(state.CANHealthy));

    ledBuff.setLED(44, boolColor(state.armCorrectOrientation));
    ledBuff.setLED(43, boolColor(state.armCorrectOrientation));

    ledBuff.setLED(42, boolColor(state.brakeMode));
    ledBuff.setLED(41, boolColor(state.brakeMode));

    ledBuff.setLED(40, boolColor(!state.intakeHasCoral));
    ledBuff.setLED(39, boolColor(!state.intakeHasCoral));
  }

  public void disabledAnimations() {
    if (Robot.getInstance().getWasEnabledThenDisabled() && lightsTimer.get() <= 5.0) {
      disabledRainbow.applyTo(rightSegment);
      disabledRainbow.applyTo(leftSegment);
      lightsTimer.restart();
    } else if (Robot.getInstance().getWasEnabledThenDisabled() && lightsTimer.get() > 5.0) {
      Robot.getInstance().setWasEnabledThenDisabled(false);
      lightsTimer.restart();
    } else if (lightsTimer.get() <= 2.0) {
      progressBarAnimation += 0.02;
      if (progressBarAnimation >= 1.0) {
        progressBarAnimation = 0.0;
      }
      LEDPattern progressBarPattern =
          LEDPattern.solid(LedColors.LightBlue.color)
              .mask(LEDPattern.progressMaskLayer(() -> progressBarAnimation));
      progressBarPattern.applyTo(rightSegment);
      progressBarPattern.applyTo(leftSegment);
    } else if (lightsTimer.get() % 10.0 <= 1.0) {
      nuclearRats(2.0, LedColors.TotalBlack, LedColors.LightBlue);
    }
  }

  public void nuclearRats(double frequency, LedColors baseColor, LedColors overlayColor) {
    LEDPattern overlayStepsPattern =
        LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous, baseColor.color, overlayColor.color);
    LEDPattern overlayStepsFinal =
        overlayStepsPattern.scrollAtRelativeSpeed(Frequency.ofBaseUnits(frequency, Units.Hertz));

    LEDPattern overlayCenterMask = LEDPattern.solid(baseColor.color);
    LEDPattern overlayCenterFlashed =
        overlayCenterMask.blink(
            Time.ofBaseUnits(1.0 / frequency, Units.Seconds),
            Time.ofBaseUnits(1.0 / frequency, Units.Seconds));
    LEDPattern overlayFinalPattern = overlayCenterMask.overlayOn(overlayCenterFlashed);

    overlayStepsFinal.applyTo(leftSegment);
    overlayStepsFinal.applyTo(rightSegment);
    if (baseColor != LedColors.TotalBlack) overlayFinalPattern.applyTo(crossSegment);
  }

  // Overload for default parameters (Kotlin default args)
  public void nuclearRats() {
    nuclearRats(4.0, LedColors.TotalBlack, LedColors.GoodGreen);
  }

  public void nuclearRats(double frequency, LedColors overlayColor) {
    nuclearRats(frequency, LedColors.TotalBlack, overlayColor);
  }

  public void blinkGreen() {
    blinkyPattern.applyTo(ledBuff);
  }

  public void setCrossbarColor(LedColors c) {
    c.solidPattern.applyTo(crossSegment);
  }
}
