package org.chillout1778.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;
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
        Handoff(Units.inchesToMeters(33.25)),
        SourceIntake(Units.inchesToMeters(53.0)),
        PreScore(Units.inchesToMeters(20.0)),
        Trough(Units.inchesToMeters(38.0)),
        L2(Units.inchesToMeters(15.0)),
        L3(Units.inchesToMeters(15.0) + Units.inchesToMeters(15.8701)),
        L4(Units.inchesToMeters(54.5 - 0.125)),
        Barge(Units.inchesToMeters(55.0 - 0.125)),
        ScoreL4(Units.inchesToMeters(54.5 - 0.125) - Units.inchesToMeters(1.0)),
        ScoreL3(Units.inchesToMeters(15.0) + Units.inchesToMeters(15.8701) - Units.inchesToMeters(3.5)),
        ScoreL2(Units.inchesToMeters(15.0) - Units.inchesToMeters(3.5)),
        PostL3(Units.inchesToMeters(15.0) - Units.inchesToMeters(6.0)),
        PostL2(Units.inchesToMeters(15.0) - Units.inchesToMeters(3.5)),
        AutoAlgae(Units.inchesToMeters(21.75)),
        LowAlgae(Units.inchesToMeters(22.25)),
        HighAlgae(Units.inchesToMeters(22.25) + Units.inchesToMeters(15.8701)),
        Processor(Units.inchesToMeters(20.0)),
        AlgaeRest(Units.inchesToMeters(15.0)),
        GroundAlgaeIntake(0.14),
        PopsiclePickup(0.065);

        public final double rawExtension;

        State(double rawExtension) {
            this.rawExtension = rawExtension;
        }

        public double getExtension() {
            return rawExtension; // Simplified version
        }
    }

    private final TalonFX mainMotor;
    private final TalonFX followerMotor;
    public boolean isZeroed = false;
    public boolean atSetpoint = false;
    public State state = State.Down; // Current state

    private Elevator() {
        mainMotor = new TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR);
        followerMotor = new TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR);

        // Apply motor configurations
        mainMotor.getConfigurator().apply(Constants.Elevator.getMotorConfig());

        followerMotor.setControl(new Follower(Constants.CanIds.ELEVATOR_MAIN_MOTOR, false));
    }

    public double getHeight() {
        return mainMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.Elevator.SPOOL_RADIUS;
    }

    public double getStatorCurrent() {
        return mainMotor.getStatorCurrent().getValueAsDouble();
    }

    public void zero() {
        mainMotor.setPosition(0.0);
        isZeroed = true;
    }

    public void setZeroingVoltage() {
        mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE);
    }

    public void stop() {
        mainMotor.setVoltage(0.0);
    }

    public void setCoastEnabled(boolean enabled) {
        NeutralModeValue mode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        mainMotor.setNeutralMode(mode);
        followerMotor.setNeutralMode(mode);
    }

    public void setState(State newState) {
        if (!isZeroed)
            return;

        this.state = newState;
        double targetPosition = newState.getExtension() / (2 * Math.PI * Constants.Elevator.SPOOL_RADIUS);
        mainMotor.setControl(new MotionMagicVoltage(targetPosition));

        // Update atSetpoint
        atSetpoint = Math.abs(getHeight() - newState.getExtension()) < Constants.Elevator.SETPOINT_THRESHOLD;
    }

    // Add getter for atSetpoint to match public field usage
    public boolean getAtSetpoint() {
        return atSetpoint;
    }

    @Override
    public void periodic() {
        // Update atSetpoint periodically
        // This would need the current target state to be accurate
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addDoubleProperty("Stator Current", this::getStatorCurrent, null);
        builder.addBooleanProperty("At Setpoint", () -> atSetpoint, null);
        builder.addBooleanProperty("Is Zeroed", () -> isZeroed, null);
        Utils.addClosedLoopProperties("Elevator Main", mainMotor, builder);
    }
}
