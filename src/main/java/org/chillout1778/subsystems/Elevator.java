package org.chillout1778.subsystems;

import static org.chillout1778.Utils.mirrorIfRed;
import static org.chillout1778.Utils.wrapTo0_2PI;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;
import org.chillout1778.Robot;
import org.chillout1778.Utils;

public class Elevator extends SubsystemBase {
  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  public enum State {
    Down(0.0),
    PreHandoff(Units.inchesToMeters(36.0)),
    Handoff(Units.inchesToMeters(34)),
    SourceIntake(Units.inchesToMeters(53.0)),
    PreScore(Units.inchesToMeters(20.0)),
    Trough(Units.inchesToMeters(40.0)),
    L2(Units.inchesToMeters(15.0)),
    L3(L2.rawExtension + Units.inchesToMeters(15.8701)),
    L4(Units.inchesToMeters(54.5 - 0.125)),
    Barge(Units.inchesToMeters(55.0 - 0.125)),
    ScoreL4(L4.rawExtension - Units.inchesToMeters(1.0)),
    ScoreL3(L3.rawExtension - Units.inchesToMeters(3.5)),
    ScoreL2(L2.rawExtension - Units.inchesToMeters(3.5)),
    PostL3(L2.rawExtension - Units.inchesToMeters(6.0)), // TODO: Tune
    PostL2(L2.rawExtension - Units.inchesToMeters(3.5)), // TODO: Tune
    AutoAlgae(Units.inchesToMeters(21.75)),
    LowAlgae(Units.inchesToMeters(21.75)),
    HighAlgae(LowAlgae.rawExtension + Units.inchesToMeters(15.8701)),
    Processor(Units.inchesToMeters(20.0)),
    AlgaeRest(Units.inchesToMeters(15.0)),
    GroundAlgaeIntake(0.14),
    PopsiclePickup(0.065);

    private final double rawExtension;

    State(double rawExtension) {
      this.rawExtension = rawExtension;
    }

    public double getExtension() {
      if (this == AutoAlgae) {
        return rawExtension + getInstance().getPreferredAlgaeHeight().getOffset();
      } else {
        return rawExtension;
      }
    }
  }

  private final TalonFX mainMotor;
  private final TalonFX followerMotor;

  private boolean isZeroed = false;

  public void zero() {
    mainMotor.setPosition(0.0);
    isZeroed = true;
  }

  public boolean getIsZeroed() {
    return isZeroed;
  }

  public void setIsZeroed(boolean isZeroed) {
    this.isZeroed = isZeroed;
  }

  public double getStatorCurrent() {
    return mainMotor.getStatorCurrent().getValueAsDouble();
  }

  private final InterpolatingDoubleTreeMap armToElevator = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap armToElevatorWhenIntakeDown =
      new InterpolatingDoubleTreeMap();

  public State state = State.Down;

  public double lastClampedSetpointForLogging = 0.0;

  public double clampSetpoint(double s) {
    long startTime = System.currentTimeMillis();
    double armDesiredPositionSignum = Math.signum(Arm.getInstance().getDesiredPosition());
    double armPosition = Arm.getInstance().getPosition();
    double interpolationTableInput =
        Math.PI
            - Math.abs(
                MathUtil.angleModulus(
                    (Math.signum(armPosition) != armDesiredPositionSignum)
                        ? 0.0
                        : ((armPosition < 0.0
                                    && Arm.getInstance().getDesiredPosition() > armPosition)
                                || (armPosition > 0.0
                                    && Arm.getInstance().getDesiredPosition() < armPosition))
                            ? Arm.getInstance().getDesiredPosition()
                            : armPosition));

    double minHeight =
        (Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Down
                && Intake.getInstance().isAtSetpoint())
            ? armToElevatorWhenIntakeDown.get(interpolationTableInput)
            : armToElevator.get(interpolationTableInput);

    double ret = MathUtil.clamp(s, minHeight, Constants.Elevator.MAX_EXTENSION);
    lastClampedSetpointForLogging = ret;

    long time = System.currentTimeMillis() - startTime;
    if (time > 5) {
      System.out.println("Elevator.clampSetpoint() took " + time + " ms");
    }
    return ret;
  }

  public double getHeight() {
    return mainMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return mainMotor.getVelocity().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return Math.abs(getHeight() - state.getExtension()) < Constants.Elevator.SETPOINT_THRESHOLD;
  }

  public boolean isLazierAtSetpoint() {
    return Math.abs(getHeight() - state.getExtension())
        < Constants.Elevator.LAZIER_SETPOINT_THRESHOLD;
  }

  public boolean isAtOrAboveSetpoint() {
    return (getHeight() + Constants.Elevator.SETPOINT_THRESHOLD) >= state.getExtension();
  }

  public void setZeroingVoltage() {
    mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE);
  }

  public void stop() {
    mainMotor.setVoltage(0.0);
  }

  public void setCoastEnabled(boolean coast) {
    if (coast) {
      mainMotor.setNeutralMode(NeutralModeValue.Coast);
      followerMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      mainMotor.setNeutralMode(NeutralModeValue.Brake);
      followerMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void periodic() {
    if (!isZeroed || !Arm.getInstance().getIsZeroed()) {
      return;
    }
    mainMotor.setControl(
        new MotionMagicVoltage(clampSetpoint(state.getExtension())).withEnableFOC(true));
  }

  public enum AlgaeHeight {
    High(Units.inchesToMeters(15.8701)),
    Low(0.0);

    private final double offset;

    AlgaeHeight(double offset) {
      this.offset = offset;
    }

    public double getOffset() {
      return offset;
    }
  }

  public Translation2d getEndOfManipulatorPose() {
    return new Translation2d(Constants.Arm.CORAL_CENTER_OFFSET, 0.0)
        .rotateBy(SwerveNext.getInstance().getEstimatedPose().getRotation())
        .plus(SwerveNext.getInstance().getEstimatedPose().getTranslation());
  }

  public AlgaeHeight getPreferredAlgaeHeight() {
    double degreesAroundReefCenter =
        getEndOfManipulatorPose()
            .minus(mirrorIfRed(Constants.Field.BLUE_REEF_CENTER))
            .getAngle()
            .getDegrees();
    if (Robot.getInstance().isRedAlliance()) {
      degreesAroundReefCenter += 180.0;
    }
    double algaeDirection =
        Math.toDegrees(wrapTo0_2PI(Math.toRadians(degreesAroundReefCenter - 30.0)));
    if ((300.0 < algaeDirection && algaeDirection < 360.0)
        || (180.0 < algaeDirection && algaeDirection < 240.0)
        || (60.0 < algaeDirection && algaeDirection < 120.0)) {
      return AlgaeHeight.Low;
    } else {
      return AlgaeHeight.High;
    }
  }

  public void setState(State state) {
    this.state = state;
  }

  private Elevator() {
    mainMotor = new TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR);
    mainMotor.getConfigurator().apply(Constants.Elevator.getMotorConfig());

    followerMotor = new TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR);
    followerMotor.getConfigurator().apply(Constants.Elevator.getFollowerConfig());
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), true));

    for (var pair : Constants.armElevatorPairs) {
      armToElevator.put(pair.getKey(), pair.getValue() + Units.inchesToMeters(0.5));
    }

    for (var pair : Constants.armInterpolationIntakeDown) {
      armToElevatorWhenIntakeDown.put(pair.getKey(), pair.getValue() + Units.inchesToMeters(0.5));
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Height", this::getHeight, null);
    builder.addDoubleProperty("Setpoint", () -> state.getExtension(), null);
    builder.addDoubleProperty("Clamped setpoint", () -> lastClampedSetpointForLogging, null);
    builder.addDoubleProperty(
        "Motion magic setpoint (deg)",
        () -> mainMotor.getClosedLoopReference().getValueAsDouble(),
        null);
    builder.addBooleanProperty("At setpoint?", this::isAtSetpoint, null);
    builder.addBooleanProperty("Is Zeroed?", () -> isZeroed, null);
    Utils.addClosedLoopProperties("Elevator", mainMotor, builder);
  }
}
