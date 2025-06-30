package org.chillout1778.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.chillout1778.Constants;

public class SwerveModule implements Sendable {    private final String name;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;
    private final PIDController turnPID;
    private final SimpleMotorFeedforward driveFeedforward;
    
    private double commandedVolts = 0.0;
    private double commandedVelocity = 0.0;

    public SwerveModule(
        String name,
        int driveMotorID,
        int turnMotorID,
        int canCoderID,
        double encoderOffset,
        InvertedValue driveInverted,
        InvertedValue turnInverted
    ) {        this.name = name;
        this.driveMotor = new TalonFX(driveMotorID);
        this.turnMotor = new TalonFX(turnMotorID);
        this.canCoder = new CANcoder(canCoderID);
        this.turnPID = Constants.Swerve.makeTurnPID();
        this.driveFeedforward = Constants.Swerve.makeDriveFeedforward();
        
        // Configure drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.DRIVE_RATIO);
        driveConfig.MotorOutput = new MotorOutputConfigs().withInverted(driveInverted);
        driveConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(45.0)
            .withStatorCurrentLimit(80.0)
            .withStatorCurrentLimitEnable(true);
        driveMotor.getConfigurator().apply(driveConfig);

        // Configure turn motor
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(Constants.Swerve.TURN_RATIO);
        turnConfig.MotorOutput = new MotorOutputConfigs().withInverted(turnInverted);
        turnConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(45.0)
            .withStatorCurrentLimit(80.0)
            .withStatorCurrentLimitEnable(true);
        turnMotor.getConfigurator().apply(turnConfig);

        // Configure CANcoder
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        canCoder.getConfigurator().apply(magnetConfig);

        turnMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() - encoderOffset);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turnMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public String getName() {
        return name;
    }

    public double getTurnPosition() {
        return MathUtil.angleModulus(turnMotor.getPosition().getValueAsDouble() * 2 * Math.PI);
    }

    private double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.Swerve.WHEEL_RADIUS;
    }

    private double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.Swerve.WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }    public void driveState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurnPosition()));
        double goalTurnPosition = optimizedState.angle.getRadians();
        double goalDriveVelocity = optimizedState.speedMetersPerSecond * Math.cos(getTurnPosition() - goalTurnPosition);
        
        commandedVelocity = goalDriveVelocity;
        
        // Use PID control for turn motor
        turnMotor.setVoltage(turnPID.calculate(getTurnPosition(), goalTurnPosition));
        
        // Use feedforward control for drive motor  
        commandedVolts = driveFeedforward.calculate(goalDriveVelocity);
        driveMotor.setVoltage(commandedVolts);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("raw cancoder position", () -> canCoder.getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("turn position (deg)", () -> Math.toDegrees(getTurnPosition()), null);
        builder.addDoubleProperty("drive velocity (mps)", this::getDriveVelocity, null);
        builder.addDoubleProperty("drive stator current", () -> driveMotor.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("command drive voltage", () -> commandedVolts, null);
        builder.addDoubleProperty("Commanded Drive Velocity", () -> commandedVelocity, null);
    }
}
