package org.chillout1778.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;
import org.chillout1778.Controls;
import org.chillout1778.Utils;

public class Intake extends SubsystemBase {
  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  // Angles measured positive downward, and the stowed starting position is 0.0.
  public enum PivotState {
    Down(Math.toRadians(126.0)),
    Trough(Math.toRadians(25.639507)),
    Up(0.0),
    OperatorControl(0.0);

    public final double angleSetpoint;

    PivotState(double angleSetpoint) {
      this.angleSetpoint = angleSetpoint;
    }
  }

  public enum RollerState {
    In(-6.0, -10.0),
    SlowIn(-2.0, -3.0),
    TroughOut(3.25, 0.0),
    Out(8.0, 0.0),
    Off(0.0, 0.0),
    AlgaeModeIdle(0.0, 0.0),
    OperatorControl(0.0, 0.0);

    public final double rollingVolts;
    public final double centeringVoltage;

    RollerState(double rollingVolts, double centeringVoltage) {
      this.rollingVolts = rollingVolts;
      this.centeringVoltage = centeringVoltage;
    }
  }

  private PivotState realPivotState = PivotState.Up;
  private RollerState realRollerState = RollerState.Off;

  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;
  private final TalonFX centeringMotor;
  private final DigitalInput linebreak;
  private final DigitalInput linebreakFoller;

  private boolean isZeroed = false;

  private Intake() {
    pivotMotor = new TalonFX(Constants.CanIds.INTAKE_PIVOT_MOTOR);
    pivotMotor.getConfigurator().apply(Constants.Intake.getPivotConfig());

    rollerMotor = new TalonFX(Constants.CanIds.INTAKE_ROLLER_MOTOR);
    rollerMotor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
    rollerMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs().withStatorCurrentLimit(80.0)
        );

    centeringMotor = new TalonFX(Constants.CanIds.INTAKE_CENTERING_MOTOR);
    centeringMotor
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    linebreak = new DigitalInput(Constants.DioIds.INTAKE_LINEBREAK);
    linebreakFoller = new DigitalInput(Constants.DioIds.INTAKE_LINEBREAK_FOLLOW);
  }

  public boolean getIsZeroed() {
    return isZeroed;
  }

  public void setIsZeroed(boolean isZeroed) {
    this.isZeroed = isZeroed;
  }

  // Kotlin: val effectivePivotState get(): PivotState { ... }
  public PivotState getEffectivePivotState() {
    if (realPivotState != PivotState.OperatorControl) {
      return realPivotState;
    } else if (isUnsafeToGoUp()) {
      return PivotState.Down;
    } else if (hasCoral()) {
      return PivotState.Up;
    } else if (Superstructure.getInstance().inputs.wantGroundIntake) {
      return PivotState.Down;
    } else {
      return PivotState.Up;
    }
  }

  // Kotlin: val effectiveRollerState get(): RollerState { ... }
  public RollerState getEffectiveRollerState() {
    if (realRollerState != RollerState.OperatorControl) {
      return realRollerState;
    } else if (Superstructure.getInstance().inputs.wantGroundIntake) {
      return RollerState.In;
    } else if (hasCoral()) {
      return RollerState.SlowIn;
    } else {
      return RollerState.Off;
    }
  }

  // Kotlin: val angle get() = pivotMotor.position.valueAsDouble * 2*Math.PI
  public double getAngle() {
    return pivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  // Kotlin: val velocity get() = pivotMotor.velocity.valueAsDouble * 2*Math.PI
  public double getVelocity() {
    return pivotMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  // Kotlin: val hasCoral get() = !linebreak.get() || Controls.operatorController.hid.touchpadButton
  public boolean hasCoral() {
    return !(linebreak.get() || linebreakFoller.get()) || Controls.operatorController.getHID().getTouchpadButton();
  }

  // Kotlin: val atSetpoint get() = Math.abs(angle - effectivePivotState.angleSetpoint) <
  // Constants.Intake.SETPOINT_THRESHOLD
  public boolean isAtSetpoint() {
    return Math.abs(getAngle() - getEffectivePivotState().angleSetpoint)
        < Constants.Intake.SETPOINT_THRESHOLD;
  }

  // Kotlin: var isZeroed: Boolean = false
  public boolean isZeroed() {
    return isZeroed;
  }

  public void zero() {
    pivotMotor.setPosition(0.0); // reset relative encoder
    // 与 Kotlin 保持一致，不再调用 setControl(new VoltageOut(0.0))
    isZeroed = true;
  }

  public void setZeroingVoltage() {
    pivotMotor.setVoltage(Constants.Intake.ZERO_VOLTAGE);
  }

  public void stop() {
    pivotMotor.setVoltage(0.0);
    rollerMotor.setVoltage(0.0);
    centeringMotor.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    if (!isZeroed) {
      return;
    }
    pivotMotor.setControl(
        new MotionMagicVoltage(getEffectivePivotState().angleSetpoint / (2 * Math.PI)));
    rollerMotor.setVoltage(getEffectiveRollerState().rollingVolts);
    centeringMotor.setVoltage(getEffectiveRollerState().centeringVoltage);
  }

  // Kotlin: fun setState(p: PivotState, r: RollerState)
  public void setState(PivotState p, RollerState r) {
    realPivotState = p;
    if (hasCoral() && r == RollerState.Off) {
      if (Controls.superstructureInputs().wantedScoringLevel
          != Superstructure.ScoringLevel.TROUGH) {
        realRollerState = RollerState.In;
      } else {
        realRollerState = RollerState.SlowIn;
      }
    } else if (hasCoral() && r == RollerState.AlgaeModeIdle) {
      realRollerState = RollerState.SlowIn;
    } else {
      realRollerState = r;
    }
  }

  // Kotlin: private val unsafeToGoUp: Boolean get() { ... }
  public boolean isUnsafeToGoUp() {
    return Math.abs(MathUtil.angleModulus(Arm.getInstance().getPosition()))
        < Math.PI - Arm.getInstance().elevatorToArm.get(Elevator.getInstance().getHeight());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Intake angle", () -> Math.toDegrees(getAngle()), null);
    builder.addDoubleProperty(
        "Intake setpoint", () -> Math.toDegrees(getEffectivePivotState().angleSetpoint), null);
    builder.addBooleanProperty("at setpoint?", this::isAtSetpoint, null);
    builder.addBooleanProperty("Intake have coral", this::hasCoral, null);
    builder.addStringProperty(
        "Effective intake pivot state", () -> getEffectivePivotState().toString(), null);
    builder.addStringProperty(
        "Underlying intake pivot state", () -> realPivotState.toString(), null);
    builder.addBooleanProperty("Is Zeroed?", this::isZeroed, null);
    Utils.addClosedLoopProperties("Intake Pivot", pivotMotor, builder);
    Utils.addClosedLoopProperties("Intake Roller", rollerMotor, builder);
    builder.addBooleanProperty("unsafe for intake to go up?", this::isUnsafeToGoUp, null);
    builder.addStringProperty("Effective intake roller state", () -> getEffectiveRollerState().toString(), null);
    builder.addStringProperty("Underlying intake roller state", () -> realRollerState.toString(), null);
  }
}
