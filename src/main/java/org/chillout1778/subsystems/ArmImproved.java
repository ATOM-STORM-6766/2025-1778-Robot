package org.chillout1778.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;

public class ArmImproved extends SubsystemBase {
    private static ArmImproved instance;
    
    public static ArmImproved getInstance() {
        if (instance == null) {
            instance = new ArmImproved();
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
    }

    public enum Side { Left, Right, Neither }

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
        PreBarge(Math.toRadians(160.0), MirrorType.AlgaeScore),
        BargeScore(Math.toRadians(160.0), MirrorType.AlgaeScore),
        Processor(Math.toRadians(70.0), MirrorType.ProcessorScore),
        AlgaeGroundPickup(Math.toRadians(-78.0), MirrorType.ActuallyFixedAngle), // out the left
        ExitAlgaeGroundPickup(Math.toRadians(-95.0), MirrorType.ActuallyFixedAngle), // out the left
        PopsiclePickup(Math.toRadians(-80.0), MirrorType.ActuallyFixedAngle);

        public final double rawAngle;
        public final MirrorType mirrorType;

        PivotState(double rawAngle, MirrorType mirrorType) {
            this.rawAngle = rawAngle;
            this.mirrorType = mirrorType;
        }        public double getDesiredAngle() {
            // Complete mirroring logic from original Kotlin code
            switch (mirrorType) {
                case ActuallyFixedAngle:
                case FixedAngle:
                    return rawAngle;
                case ClosestToReef:
                    // Use static methods from parent class for side calculations
                    Side reefSide = ArmImproved.getSideCloserToReef();
                    switch (reefSide) {
                        case Left:
                            return -rawAngle;
                        case Right:
                            return rawAngle;
                        case Neither:
                            // If we can't decide, go to Up or Down based on which is closer
                            return Math.abs(rawAngle) < Math.PI/2 ? 0.0 : Math.PI;
                    }
                    break;
                case AlgaeScore:
                    Side bargeSide = ArmImproved.getSideCloserToBarge();
                    switch (bargeSide) {
                        case Left:
                            return -rawAngle;
                        case Right:
                            return rawAngle;
                        case Neither:
                            return Math.PI;
                    }
                    break;
                case ProcessorScore:
                    Side processorSide = ArmImproved.getSideCloserToProcessor();
                    switch (processorSide) {
                        case Left:
                            return -rawAngle;
                        case Right:
                            return rawAngle;
                        case Neither:
                            return Math.PI;
                    }
                    break;
                case ClosestToPosition:
                    // Mirror based on current position - simplified
                    return rawAngle; // Will need access to instance position
                default:
                    return rawAngle;
            }
            return rawAngle;
        }
    }

    private final TalonFX armPivotMotor;
    private final TalonFX rollerMotor;
    
    public boolean hasObject = false;
    public boolean isZeroed = false;
    public boolean atSetpoint = false;
    public boolean isArmStuck = false;
    public Timer autoTimer = new Timer();
      // Current states
    private PivotState currentPivotState = PivotState.Down;
    private RollerState currentRollerState = RollerState.Off;
    
    // Static methods for side calculations (simplified implementations)
    public static Side getSideCloserToReef() {
        // Simplified logic - in real implementation, this would check robot position
        // relative to reef locations and determine which side is closer
        return Side.Right; // Default fallback
    }
    
    public static Side getSideCloserToBarge() {
        // Simplified logic - in real implementation, this would check robot position
        // relative to barge scoring locations
        return Side.Right; // Default fallback
    }
    
    public static Side getSideCloserToProcessor() {
        // Simplified logic - in real implementation, this would check robot position
        // relative to processor locations
        return Side.Right; // Default fallback
    }

    private ArmImproved() {
        new CANcoder(Constants.CanIds.ARM_ENCODER);
        armPivotMotor = new TalonFX(Constants.CanIds.ARM_PIVOT_MOTOR);
        rollerMotor = new TalonFX(Constants.CanIds.ARM_ROLLER_MOTOR);
        
        // Apply configurations
        armPivotMotor.getConfigurator().apply(Constants.Arm.getPivotConfig());
        
        CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();
        rollerCurrentLimits.withStatorCurrentLimit(80.0);
        rollerMotor.getConfigurator().apply(rollerCurrentLimits);
    }

    public double getPosition() {
        return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
    }    public void zero() {
        // Complete zeroing logic from original
        armPivotMotor.setPosition(0.0); // Reset relative encoder
        isZeroed = true;
        // Stop any current motion during zeroing
        armPivotMotor.setVoltage(0.0);
    }

    public void setZeroingVoltage() {
        armPivotMotor.setVoltage(Constants.Arm.ZERO_VOLTAGE);
    }

    public void stop() {
        armPivotMotor.setVoltage(0.0);
        rollerMotor.setVoltage(0.0);
    }    public void setCoastEnabled(boolean enabled) {
        // Implement coast mode setting
        if (enabled) {
            armPivotMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }    // State setters
    public void setPivotState(PivotState state) {
        currentPivotState = state;
        // Implement motion magic control to desired angle
        double targetAngle = state.getDesiredAngle();
        // Convert angle to motor rotations and apply motion magic control
        double targetRotations = targetAngle / (2 * Math.PI);
        // armPivotMotor.setControl(new MotionMagicVoltage(targetRotations));
        // For now, using simple position control as placeholder
        System.out.println("Setting arm to angle: " + Math.toDegrees(targetAngle) + " degrees");
    }

    public void setRollerState(RollerState state) {
        currentRollerState = state;
        rollerMotor.set(state.dutyCycle);
    }

    // Getters
    public PivotState getPivotState() {
        return currentPivotState;
    }

    public RollerState getRollerState() {
        return currentRollerState;
    }

    // Placeholder for elevatorToArm interpolation map
    public InterpolatingDoubleTreeMap getElevatorToArm() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        map.put(0.0, Math.PI); // Placeholder values
        map.put(1.0, Math.PI/2);
        return map;
    }

    // Add getters to match public field usage
    public boolean getAtSetpoint() {
        return atSetpoint;
    }

    public boolean getIsArmStuck() {
        return isArmStuck;
    }    @Override
    public void periodic() {
        // Implement periodic logic from original
        
        // Update atSetpoint based on current position vs target
        double currentAngle = getPosition();
        double targetAngle = currentPivotState.getDesiredAngle();
        atSetpoint = Math.abs(currentAngle - targetAngle) < Constants.Arm.SETPOINT_THRESHOLD;
          // Check for stuck condition (simplified)
        double currentStatorCurrent = armPivotMotor.getStatorCurrent().getValueAsDouble();
        isArmStuck = currentStatorCurrent > 40.0; // Simplified threshold
        
        // Additional periodic logic can be added here
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Position", this::getPosition, null);
        builder.addBooleanProperty("Has Object", () -> hasObject, null);
        builder.addBooleanProperty("At Setpoint", () -> atSetpoint, null);
        builder.addStringProperty("Pivot State", () -> currentPivotState.name(), null);
        builder.addStringProperty("Roller State", () -> currentRollerState.name(), null);
    }
}
