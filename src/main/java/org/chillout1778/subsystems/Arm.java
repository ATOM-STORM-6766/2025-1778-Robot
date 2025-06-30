package org.chillout1778.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;

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
        SlowIdle(-0.035),
        FastIdle(-0.1),
        Idle(-0.035),
        AlgaeIdle(-0.225),
        In(-1.0),
        SlowOut(0.075),
        Out(1.0),
        Descore(0.8);

        public final double dutyCycle;

        RollerState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return dutyCycle;
        }
    }

    public enum PivotState {
        Up(Math.PI),
        AlgaeUp(Math.PI),
        Down(0.0),
        ScoreCoral(Math.toRadians(130.0)),
        FinishScoreCoral(Math.toRadians(105.0)),
        AboveScoreCoral(Math.toRadians(160.0)),
        L4ScoreCoral(Math.toRadians(135.0)),
        L4FinishScoreCoral(Math.toRadians(100.0)),
        GetAlgae(Math.toRadians(100.0)),
        PostAlgae(Math.toRadians(110.0)),
        DescoreAlgae(Math.toRadians(110.0)),
        SafeInsideRobotAngle(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE),
        PreBarge(Math.toRadians(160.0)),
        BargeScore(Math.toRadians(160.0)),
        Processor(Math.toRadians(70.0)),
        AlgaeGroundPickup(Math.toRadians(-78.0)),
        ExitAlgaeGroundPickup(Math.toRadians(-95.0)),
        PopsiclePickup(Math.toRadians(-80.0));

        private final double rawAngle;

        PivotState(double rawAngle) {
            this.rawAngle = rawAngle;
        }

        public double getRawAngle() {
            return rawAngle;
        }

        // Simplified angle calculation - full mirror logic would be more complex
        public double getDesiredAngle() {
            return rawAngle; // Simplified implementation
        }
    }

    public enum Side {
        Left, Right, Neither
    };

    private final DutyCycleEncoder absoluteEncoder;
    private final TalonFX armPivotMotor;
    private final TalonFX rollerMotor;

    public boolean hasObject = false;
    public boolean isZeroed = false;
    public boolean atSetpoint = false;
    public boolean isArmStuck = false;
    public Timer autoTimer = new Timer();

    private RollerState rollerState = RollerState.Off;

    private Arm() {
        absoluteEncoder = new DutyCycleEncoder(Constants.DioIds.ARM_ABSOLUTE_ENCODER);
        armPivotMotor = new TalonFX(Constants.CanIds.ARM_PIVOT_MOTOR);
        rollerMotor = new TalonFX(Constants.CanIds.ARM_ROLLER_MOTOR);

        // Apply motor configurations
        armPivotMotor.getConfigurator().apply(Constants.Arm.getPivotConfig());
    }

    public double getPosition() {
        return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    private double getCloseClampedPosition() {
        // DutyCycleEncoder returns position as a value between 0.0 and 1.0 (rotations)
        double encoderValue = absoluteEncoder.get();
        
        // Convert to the expected format and apply offset
        double x = encoderValue - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS;
        
        // Normalize to [-0.5, 0.5] range
        while (x < -0.5)
            x += 1.0;
        while (x > 0.5)
            x -= 1.0;
            
        double rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO;

        double allowedOffsetArmRotations = 12.0 / 360.0;

        // Hack
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

    public void zero() {
        // Basic zeroing implementation - set to known safe position
        resetRelativeFromAbsolute();
        isZeroed = true;
    }

    public void setZeroingVoltage() {
        armPivotMotor.setVoltage(Constants.Arm.ZERO_VOLTAGE);
    }

    public void stop() {
        armPivotMotor.setVoltage(0.0);
        rollerMotor.setVoltage(0.0);
    }

    public void setState(PivotState pivot, RollerState roller) {
        this.rollerState = roller;
    }

    public void setCoastEnabled(boolean coast) {
        if (coast) {
            armPivotMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }// Placeholder for elevatorToArm interpolation map

    public InterpolatingDoubleTreeMap getElevatorToArm() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        map.put(0.0, Math.PI); // Placeholder values
        map.put(1.0, Math.PI / 2);
        return map;
    }

    // Add getters to match public field usage
    public boolean getAtSetpoint() {
        return atSetpoint;
    }

    public boolean getIsArmStuck() {
        return isArmStuck;
    }

    @Override
    public void periodic() {
        // Update object detection
        double currentStatorCurrent = armPivotMotor.getStatorCurrent().getValueAsDouble();

        // Simple object detection based on current threshold
        boolean undebouncedHasObject = currentStatorCurrent > 15.0; // Threshold from Kotlin version

        // For safety and simplicity, just use the raw signal for now
        hasObject = undebouncedHasObject;

        // Set roller motor speed based on current state
        if (isZeroed) {
            rollerMotor.set(rollerState.getDutyCycle());
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm");
        builder.addDoubleProperty("Position", this::getPosition, null);
        builder.addBooleanProperty("Has Object", () -> hasObject, null);
        builder.addBooleanProperty("At Setpoint", () -> atSetpoint, null);
        builder.addBooleanProperty("Is Zeroed", () -> isZeroed, null);
        builder.addBooleanProperty("Is Arm Stuck", () -> isArmStuck, null);
    }
}
