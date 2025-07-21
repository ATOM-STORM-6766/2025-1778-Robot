package org.chillout1778.subsystems;

import static org.chillout1778.Constants.Field.BLUE_REEF_CENTER;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import org.chillout1778.Constants;
import org.chillout1778.Controls;
import org.chillout1778.Robot;
import org.chillout1778.Utils;

public class Arm extends SubsystemBase {
  private static Arm instance;

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public enum RollerState {
    Off(0.0),
    SlowIdle(-0.42),
    FastIdle(-1.2),
    Idle(-0.42),
    AlgaeIdle(-2.7),
    In(-9.0),
    SlowOut(1.4),
    Out(12.0),
    Descore(9.6);

    public final double voltage;

    RollerState(double voltage) {
      this.voltage = voltage;
    }
  }

  public enum MirrorType {
    FixedAngle,
    ActuallyFixedAngle,
    ClosestToReef,
    ClosestToPosition,
    AlgaeScore,
    ProcessorScore
  }

  public enum PivotState {
    Up(Math.PI, MirrorType.FixedAngle),
    AlgaeUp(Math.PI, MirrorType.FixedAngle),
    Down(0.0, MirrorType.ActuallyFixedAngle),
    ScoreCoral(Math.toRadians(130.0), MirrorType.ClosestToReef),
    FinishScoreCoral(Math.toRadians(105.0), MirrorType.ClosestToReef),
    AboveScoreCoral(Math.toRadians(160.0), MirrorType.ClosestToReef),
    L4ScoreCoral(Math.toRadians(135.0), MirrorType.ClosestToReef),
    L4FinishScoreCoral(Math.toRadians(100.0), MirrorType.ClosestToReef),
    GetAlgae(Math.toRadians(100.0), MirrorType.ClosestToReef),
    PostAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
    DescoreAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
    SafeInsideRobotAngle(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE, MirrorType.ClosestToReef),
    PreBarge(Math.toRadians(165.0), MirrorType.AlgaeScore),
    BargeScore(Math.toRadians(165.0), MirrorType.AlgaeScore),
    Processor(Math.toRadians(80.0), MirrorType.ProcessorScore),
    AlgaeGroundPickup(Math.toRadians(-78.0), MirrorType.ActuallyFixedAngle),
    ExitAlgaeGroundPickup(Math.toRadians(-95.0), MirrorType.ActuallyFixedAngle),
    PopsiclePickup(Math.toRadians(-80.0), MirrorType.ActuallyFixedAngle);

    private final double rawAngle;
    public final MirrorType mirrorType;

    PivotState(double rawAngle, MirrorType mirrorType) {
      this.rawAngle = rawAngle;
      this.mirrorType = mirrorType;
    }

    public double getDesiredAngle() {
      switch (mirrorType) {
        case ActuallyFixedAngle:
        case FixedAngle:
          return rawAngle;
        case ClosestToPosition:
          return Arm.getInstance().getPosition() > 0.0 ? rawAngle : -rawAngle;
        case ClosestToReef:
          switch (Arm.getInstance().getSideCloserToReef()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.abs(rawAngle) < Math.PI / 2 ? 0.0 : Math.PI;
          }
        case AlgaeScore:
          switch (Arm.getInstance().getSideCloserToBarge()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.PI;
          }
        case ProcessorScore:
          switch (Arm.getInstance().getSideCloserToProcessor()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.PI;
          }
      }
      return rawAngle; // Default
    }
  }

  private final DutyCycleEncoder absoluteEncoder;
  private final TalonFX armPivotMotor;
  private final TalonFX rollerMotor;
  private final StatusSignal<Current> statorCurrentSignal;

  public enum Side {
    Left,
    Right,
    Neither
  }

  private final double deadzoneAngle = Math.toRadians(20.0);

  public TalonFX getArmPivotMotor() {
    return armPivotMotor;
  }

  public TalonFX getRollerMotor() {
    return rollerMotor;
  }

  private Side getSideCloserToReef() {
    Rotation2d directionTowardReefCenter =
        Utils.mirrorIfRed(BLUE_REEF_CENTER)
            .minus(SwerveNext.getInstance().getEstimatedPose().getTranslation())
            .getAngle();
    Rotation2d directionTowardRight =
        SwerveNext.getInstance().getEstimatedPose().getRotation().rotateBy(Rotation2d.kCW_90deg);

    double ang = angleBetween(directionTowardReefCenter, directionTowardRight);
    assert (ang >= 0.0);

    if (Math.PI / 2 - deadzoneAngle < ang && ang < Math.PI / 2 + deadzoneAngle) {
      return Side.Neither;
    } else if (ang < Math.PI / 2) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  private double angleBetween(Rotation2d r1, Rotation2d r2) {
    return Math.acos(r1.getCos() * r2.getCos() + r1.getSin() * r2.getSin());
  }

  private Side getSideCloserToBarge() {
    boolean isOnBlue = !Robot.getInstance().isOnRedSide();
    double rotation = SwerveNext.getInstance().getEstimatedPose().getRotation().getRadians();

    if ((rotation < Math.PI && rotation > Math.PI - deadzoneAngle)
        || (rotation > -Math.PI && rotation < -Math.PI + deadzoneAngle)
        || (rotation > -deadzoneAngle && rotation < deadzoneAngle)) {
      return Side.Neither;
    } else if ((isOnBlue && rotation > 0.0) || (!isOnBlue && rotation < 0.0)) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  private Side getSideCloserToProcessor() {
    boolean isOnBlue = !Robot.getInstance().isOnRedSide();
    double rotation = SwerveNext.getInstance().getEstimatedPose().getRotation().getRadians();

    if ((rotation < (Math.PI / 2 + deadzoneAngle) && rotation > (Math.PI / 2 - deadzoneAngle))
        || (rotation > (-Math.PI / 2 - deadzoneAngle)
            && rotation < (-Math.PI / 2 + deadzoneAngle))) {
      return Side.Neither;
    } else if ((isOnBlue && rotation < Math.PI / 2 && rotation > -Math.PI / 2)
        || (!isOnBlue && (rotation > Math.PI / 2 || rotation < -Math.PI / 2))) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public boolean atSafeReefDistance() {
    return SwerveNext.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(Utils.mirrorIfRed(BLUE_REEF_CENTER))
        > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER;
  }

  public boolean atSafePlacementDistance() {
    return SwerveNext.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(Utils.mirrorIfRed(BLUE_REEF_CENTER))
        > Constants.Arm.SAFE_PLACEMENT_DISTANCE;
  }

  public boolean atSafeBargeDistance() {
    return SwerveNext.getInstance().getEstimatedPose().getX()
            < Constants.Field.FIELD_X_SIZE / 2 - Constants.Arm.SAFE_BARGE_DISTANCE
        || SwerveNext.getInstance().getEstimatedPose().getX()
            > Constants.Field.FIELD_X_SIZE / 2 + Constants.Arm.SAFE_BARGE_DISTANCE;
  }

  public boolean atSafeProcessorDistance() {
    return SwerveNext.getInstance().getEstimatedPose().getY() > Constants.Field.SAFE_WALL_DISTANCE
        && SwerveNext.getInstance().getEstimatedPose().getY()
            < (Constants.Field.FIELD_Y_SIZE - Constants.Field.SAFE_WALL_DISTANCE);
  }

  public boolean isInsideFrame() {
    double position = getPosition();
    return Math.abs(MathUtil.angleModulus(position)) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE
        || Math.abs(MathUtil.angleModulus(position))
            > (Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
  }

  public final InterpolatingDoubleTreeMap elevatorToArm = new InterpolatingDoubleTreeMap();
  public final InterpolatingDoubleTreeMap elevatorToArmWhenIntakeDown =
      new InterpolatingDoubleTreeMap();

  private boolean isArmStuck = false;

  public boolean getIsArmStuck() {
    return isArmStuck;
  }

  private double positionFromAngle(double angle, boolean respectReef) {
    List<Double> positions =
        Arrays.asList(Utils.wrapTo0_2PI(angle), Utils.wrapTo0_2PI(angle) - 2 * Math.PI).stream()
            .filter(
                p ->
                    p >= Constants.Arm.ALLOWED_OPERATING_RANGE_MIN
                        && p <= Constants.Arm.ALLOWED_OPERATING_RANGE_MAX)
            .collect(Collectors.toList());

    double actualArmPosition = getPosition();
    Side closeSide = getSideCloserToReef();

    double p;
    if (positions.size() == 1) {
      p = positions.get(0);
    } else if (respectReef) {
      switch (closeSide) {
        case Neither:
          p =
              positions.stream()
                  .min(Comparator.comparingDouble(pos -> Math.abs(pos - actualArmPosition)))
                  .get();
          break;
        case Right:
          if (actualArmPosition < Math.PI / 2) p = positions.get(1);
          else p = positions.get(0);
          break;
        case Left:
          if (actualArmPosition > -Math.PI / 2) p = positions.get(0);
          else p = positions.get(1);
          break;
        default:
          p =
              positions.stream()
                  .min(Comparator.comparingDouble(pos -> Math.abs(pos - actualArmPosition)))
                  .get();
          break;
      }
    } else {
      p =
          positions.stream()
              .min(Comparator.comparingDouble(pos -> Math.abs(pos - actualArmPosition)))
              .get();
    }

    boolean notAtSafeReefDistance = !atSafeReefDistance();
    if (actualArmPosition > Math.PI / 2
        && p < Math.PI / 2
        && closeSide == Side.Right
        && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.max(p, Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else if (actualArmPosition < -Math.PI / 2
        && p > -Math.PI / 2
        && closeSide == Side.Left
        && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.min(p, -Math.PI + Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else {
      isArmStuck = false;
    }

    double actualElevatorHeight = Elevator.getInstance().getHeight();
    double limit =
        (Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Down
                && Intake.getInstance().isAtSetpoint())
            ? elevatorToArmWhenIntakeDown.get(actualElevatorHeight)
            : elevatorToArm.get(actualElevatorHeight);

    if (MathUtil.isNear(Math.PI, limit, 0.0001)) {
      return p;
    } else if (actualArmPosition < 0.0) {
      return MathUtil.clamp(p, -Math.PI - limit, -Math.PI + limit);
    } else {
      return MathUtil.clamp(p, Math.PI - limit, Math.PI + limit);
    }
  }

  private long lastUpdatedTick = -1;
  private double lastCachedValue = 0.0;

  public double getDesiredPosition() {
    if (lastUpdatedTick == Robot.getInstance().tickNumber) {
      return lastCachedValue;
    }
    long startTime = System.currentTimeMillis();
    double answer =
        positionFromAngle(
            pivotState.getDesiredAngle(), pivotState.mirrorType != MirrorType.ActuallyFixedAngle);
    long endTime = System.currentTimeMillis();
    lastUpdatedTick = Robot.getInstance().tickNumber;
    lastCachedValue = answer;
    if (endTime - startTime > 5) {
      System.out.println("desired position took " + (endTime - startTime) + " ms");
    }
    return answer;
  }

  private PivotState pivotState = PivotState.Up;
  private RollerState rollerState = RollerState.Off;

  private boolean isZeroed = false;
  private final Debouncer coralCurrentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeCurrentDebouncer = new Debouncer(0.4, Debouncer.DebounceType.kBoth);
  private boolean hasObject = false;
  public final Timer autoTimer = new Timer();

  public boolean getHasObject() {
    return hasObject;
  }

  public void setHasObject(boolean hasObject) {
    this.hasObject = hasObject;
  }

  private boolean getUndebouncedHasObject() {
    return (rollerState == RollerState.Idle || rollerState == RollerState.SlowIdle)
        ? statorCurrentSignal.getValueAsDouble() > Constants.Arm.IDLE_CURRENT_DRAW
        : statorCurrentSignal.getValueAsDouble() > Constants.Arm.CURRENT_DRAW;
  }

  private final double armOffsetIncrementRadians = Math.toRadians(0.1);
  private double armOffsetRadians = 0.0;

  private Arm() {
    absoluteEncoder = new DutyCycleEncoder(Constants.DioIds.ARM_ABSOLUTE_ENCODER);
    armPivotMotor = new TalonFX(Constants.CanIds.ARM_PIVOT_MOTOR);
    armPivotMotor.getConfigurator().apply(Constants.Arm.getPivotConfig());

    rollerMotor = new TalonFX(Constants.CanIds.ARM_ROLLER_MOTOR);
    rollerMotor.getConfigurator().apply(Constants.Arm.getRollerConfig());

    statorCurrentSignal = rollerMotor.getStatorCurrent();
    statorCurrentSignal.setUpdateFrequency(100.0);

    for (var entry : Constants.armElevatorPairs) {
      elevatorToArm.put(entry.getValue(), entry.getKey());
    }
    for (var entry : Constants.armInterpolationIntakeDown) {
      elevatorToArmWhenIntakeDown.put(entry.getValue(), entry.getKey());
    }
  }

  @Override
  public void periodic() {
    if (Controls.wantOffsetArmPositive()) offsetArm(armOffsetIncrementRadians);
    if (Controls.wantOffsetArmNegative()) offsetArm(-armOffsetIncrementRadians);

    boolean atStartOfAuto = (Robot.getInstance().isAutonomous() && autoTimer.get() < 0.75);
    statorCurrentSignal.refresh();
    boolean debouncedHasCoral = coralCurrentDebouncer.calculate(getUndebouncedHasObject());
    boolean debouncedHasAlgae = algaeCurrentDebouncer.calculate(getUndebouncedHasObject());
    hasObject =
        atStartOfAuto
            || (rollerState == RollerState.AlgaeIdle ? debouncedHasAlgae : debouncedHasCoral);

    if (!isZeroed || !Elevator.getInstance().getIsZeroed()) {
      return;
    }

    rollerMotor.setControl(
        new VoltageOut(atStartOfAuto ? RollerState.FastIdle.voltage : rollerState.voltage)
            .withEnableFOC(true));

    double positionSetpoint = getDesiredPosition();
    double gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(getPosition());

    armPivotMotor.setControl(
        new MotionMagicVoltage((positionSetpoint - armOffsetRadians) / (2 * Math.PI))
            .withEnableFOC(true)
            .withFeedForward(gravityFeedforward));
  }

  public double getCloseClampedPosition() {
    // DutyCycleEncoder returns position as a value between 0.0 and 1.0 (rotations)
    double x = absoluteEncoder.get() - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS;
    while (x < -0.5) x += 1.0;
    while (x > 0.5) x -= 1.0;
    double rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO;

    double allowedOffsetArmRotations = 12.0 / 360.0;

    if (Math.abs(rawReadingArmRotations - 0.5) < allowedOffsetArmRotations) {
      return 0.5;
    } else if (Math.abs(rawReadingArmRotations - (-0.5)) < allowedOffsetArmRotations) {
      return -0.5;
    } else {
      return rawReadingArmRotations;
    }
  }

  public void resetRelativeFromAbsolute() {
    armPivotMotor.setPosition(getCloseClampedPosition());
    isZeroed = true;
  }

  public double getPosition() {
    return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI + armOffsetRadians;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getDesiredPosition() - getPosition()) < Constants.Arm.SETPOINT_THRESHOLD;
  }

  public void setState(PivotState pivot, RollerState rollers) {
    this.pivotState = pivot;
    this.rollerState = rollers;
  }

  public void setCoastEnabled(boolean coast) {
    if (coast) {
      armPivotMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void offsetArm(double r) {
    armOffsetRadians += r;
  }

  public boolean getIsZeroed() {
    return isZeroed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm offset (deg)", () -> Math.toDegrees(armOffsetRadians), null);
    builder.addDoubleProperty("Arm position (deg)", () -> Math.toDegrees(getPosition()), null);
    builder.addDoubleProperty("Raw Encoder", () -> absoluteEncoder.get(), null);
    builder.addBooleanProperty("Has object?", () -> hasObject, null);
    builder.addDoubleProperty(
        "Desired position (deg)", () -> Math.toDegrees(getDesiredPosition()), null);
    builder.addDoubleProperty(
        "Motion magic setpoint (deg)",
        () -> 360.0 * armPivotMotor.getClosedLoopReference().getValueAsDouble(),
        null);
    builder.addBooleanProperty("At setpoint?", this::isAtSetpoint, null);
    builder.addDoubleProperty("Roller current", () -> statorCurrentSignal.getValueAsDouble(), null);
    builder.addBooleanProperty("Is Zeroed?", () -> isZeroed, null);
    Utils.addClosedLoopProperties("Arm pivot", armPivotMotor, builder);
  }
}
