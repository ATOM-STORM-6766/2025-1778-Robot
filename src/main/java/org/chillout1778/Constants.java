package org.chillout1778;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.ArrayList;

public class Constants {
    public static class CanIds {
        // Don't put anything on ID 0, when a new motor is recognized in Phoenix Tuner X
        // it causes issues
        public static final int SWERVE_FRONT_LEFT_DRIVE = 1;
        public static final int SWERVE_FRONT_LEFT_TURN = 2;
        public static final int SWERVE_FRONT_RIGHT_DRIVE = 3;
        public static final int SWERVE_FRONT_RIGHT_TURN = 4;
        public static final int SWERVE_BACK_RIGHT_DRIVE = 5;
        public static final int SWERVE_BACK_RIGHT_TURN = 6;
        public static final int SWERVE_BACK_LEFT_DRIVE = 7;
        public static final int SWERVE_BACK_LEFT_TURN = 8;
        public static final int SWERVE_FRONT_LEFT_CANCODER = 9;
        public static final int SWERVE_FRONT_RIGHT_CANCODER = 10;
        public static final int SWERVE_BACK_LEFT_CANCODER = 11;
        public static final int SWERVE_BACK_RIGHT_CANCODER = 12;

        public static final int ELEVATOR_MAIN_MOTOR = 13;
        public static final int ELEVATOR_FOLLOWER_MOTOR = 14;

        public static final int ARM_PIVOT_MOTOR = 15;
        public static final int ARM_ROLLER_MOTOR = 16;
        public static final int ARM_ENCODER = 20;

        public static final int INTAKE_PIVOT_MOTOR = 17;
        public static final int INTAKE_ROLLER_MOTOR = 18;
        public static final int INTAKE_CENTERING_MOTOR = 19;

        public static final int GYRO = 30;
    }

    public static class DioIds {
        public static final int ARM_ABSOLUTE_ENCODER = 0;
        public static final int INTAKE_LINEBREAK = 1;
        public static final int DISABLE_BREAK_MODE = 2;
    }

    public static class Vision {
        public static final AprilTagFields FIELD_TYPE = AprilTagFields.k2025ReefscapeWelded;

        public static final String FRONT_RIGHT_NAME = "Front Right";
        public static final String FRONT_LEFT_NAME = "Front Left";
        public static final String BACK_RIGHT_NAME = "Back Right";
        public static final String BACK_LEFT_NAME = "Back Left";

        // positive x is to the front (intake side), positive y is to the left, and
        // counterclockwise (when looking down at the robot) is positive theta
        public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
                new Translation3d(-0.012552, -0.319809, 0.191168),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-70.0)));
        public static final Transform3d BACK_RIGHT_TRANSFORM = new Transform3d(
                new Translation3d(-0.081165, -0.322330, 0.191168),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-(180.0 - 55.0))));
        public static final Transform3d FRONT_LEFT_TRANSFORM = new Transform3d(
                new Translation3d(-0.012552, 0.319809, 0.191168),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(70.0)));
        public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
                new Translation3d(-0.081165, 0.322330, 0.191168),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0 - 55.0)));
    }

    public static class Field {
        public static final double FIELD_X_SIZE = 17.548249;
        public static final double FIELD_Y_SIZE = 8.051800;

        public static final double BLUE_BARGE_SCORING_X = 7.6;
        public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;

        public static final double SAFE_WALL_DISTANCE = 1.0;

        /**
         * Coordinates of the center of the reef on the blue side of the field
         */
        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(
                Units.inchesToMeters(176.75),
                Units.inchesToMeters(158.5));

        /**
         * Distance between the center of the robot and the center of the reef
         * when the robot is at the center of one of the reef's edges.
         */
        public static final double ROBOT_REEF_CENTER_DISTANCE = Units.inchesToMeters(32.25 + 18.0);

        /**
         * Branch offset from the center of one of the reef's edges
         */
        public static final double REEF_BRANCH_OFFSET_DISTANCE = Units.inchesToMeters(12.9375 / 2);

        public static final double TROUGH_OFFSET_DISTANCE = Units.inchesToMeters(14.5 / 2);

        // Complex pose calculations would need to be implemented in static
        // initialization blocks
        // For now, leaving as placeholder
        public static final List<Pose2d> blueScoringPoses = new ArrayList<>();
        public static final List<Pose2d> blueTroughScoringPoses = new ArrayList<>();
        public static final List<Pose2d> blueAlgaeGrabbingPose = new ArrayList<>();
        public static final List<Pose2d> redScoringPoses = new ArrayList<>();
        public static final List<Pose2d> redTroughScoringPoses = new ArrayList<>();
        public static final List<Pose2d> redAlgaeGrabbingPose = new ArrayList<>();
    }

    public static class Intake {
        public static final double ZERO_MIN_CURRENT = 20.0; // Updated from Kotlin version
        public static final double SETPOINT_THRESHOLD = Math.toRadians(5.0);
        public static final double ZERO_VOLTAGE = -0.7; // Updated from Kotlin version

        private static final double GEAR_RATIO = 1.0 / ((12.0 / 40.0) * (18.0 / 46.0) * (18.0 / 60.0) * (12.0 / 32.0));

        public static TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kG = 0.0;
            config.Slot0.kP = 160.0;
            config.MotionMagic.MotionMagicJerk = 2000.0;
            config.MotionMagic.MotionMagicAcceleration = 200.0;
            config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            return config;
        }
    }

    public static class Elevator {
        public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
        public static final double GEAR_RATIO = 4.0;
        public static final double ZERO_VOLTAGE = -0.2;
        public static final double ZERO_MIN_CURRENT = 1.7; // amps
        public static final double SETPOINT_THRESHOLD = 0.01;
        public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;
        public static final double COLLISION_AVOIDANCE_MARGIN = 1.0;
        public static final double MAX_EXTENSION = Units.inchesToMeters(55.0);
        public static final double SAFE_HEIGHT = 0.837198 - 0.01;

        public static TalonFXConfiguration getMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO / (SPOOL_RADIUS * 2 * Math.PI);
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kG = 0.37;
            config.Slot0.kP = 70.0;
            config.MotionMagic.MotionMagicAcceleration = 14.0;
            config.MotionMagic.MotionMagicCruiseVelocity = 3.0;
            return config;
        }
    }

    public static class Arm {
        public static final double CORAL_CENTER_OFFSET = 0.1;
        public static final double ZERO_VOLTAGE = -2.0;
        public static final double SAFE_INSIDE_ROBOT_ANGLE = Math.toRadians(30.0);
        public static final double POSITION_DEPENDENT_KG = 0.33;
        public static final double PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS = 0.7209;
        public static final double SETPOINT_THRESHOLD = 0.1;
        public static final double PIVOT_ENCODER_RATIO = (36.0 / 18.0) * (36.0 / 18.0) * (60.0 / 24.0) * (12.0 / 54.0);

        // PID and motion magic constants
        public static final double PIVOT_GEAR_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (12.0 / 54.0);

        public static TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            config.Feedback.SensorToMechanismRatio = 1.0 / PIVOT_GEAR_RATIO;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.1;
            config.Slot0.kA = 0.0;
            config.Slot0.kG = 0.0;
            config.Slot0.kP = 80.0; // volts per rotation
            config.MotionMagic.MotionMagicJerk = 9999.0;
            config.MotionMagic.MotionMagicAcceleration = 4.5;
            config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rps
            config.CurrentLimits.StatorCurrentLimit = 70.0;
            config.CurrentLimits.SupplyCurrentLimit = 50.0;
            return config;
        }
    }

    public static class Swerve {
        public static final double ALIGNMENT_TOLERANCE = 0.04;
        public static final double STARTING_TOLERANCE = 0.15;

        // Module constants
        public static final double WHEEL_RADIUS = Units.inchesToMeters(3.9 / 2);
        public static final double DRIVE_RATIO = 1.0 / ((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
        public static final double TURN_RATIO = 150.0 / 7.0;

        // How far the swerve modules are from (0,0).
        public static final double XY_DISTANCE = Units.inchesToMeters(13.393747);

        // How fast the robot can move in a straight line (meters/sec).
        public static final double MAX_VELOCITY = (5800.0 / 60) / DRIVE_RATIO * WHEEL_RADIUS * 2 * Math.PI;

        // How fast the robot can rotate (radians/sec).
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (XY_DISTANCE * Math.sqrt(2.0));

        public static final double CHASSIS_RADIUS = XY_DISTANCE * Math.sqrt(2.0);

        // Alignment constants
        public static final double maxAlignTranslationSpeed = 1.5;
        public static final double maxAlignRotationSpeed = 2.5;
        public static final double maxBargeAlignTranslationSpeed = 1.5;
        public static final double maxBargeAlignRotationSpeed = 1.5;
        public static final double MAX_NODE_DISTANCE = 3.0; // meters

        public static final double ALIGN_ANGLE_WEIGHT = 2.7;
        public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
        public static final double ALREADY_SCORED_BADNESS = 0.5
                + Units.inchesToMeters(12.9375) * ALIGN_TRANSLATION_WEIGHT * (1 - 2 * 0.3);

        // Encoder offsets - from original Kotlin
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.44; // ROTATIONS
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.276; // ROTATIONS
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.372; // ROTATIONS
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.298; // ROTATIONS

        // PID Controllers factory methods
        public static PIDController makeTurnPID() {
            PIDController pid = new PIDController(7.0, 0.0, 0.01);
            pid.enableContinuousInput(-Math.PI, Math.PI);
            return pid;
        }

        public static SimpleMotorFeedforward makeDriveFeedforward() {
            return new SimpleMotorFeedforward(0.2199442, 2.18943902193, 0.0);
        }

        public static PIDController makeAlignTurnPID() {
            PIDController pid = new PIDController(6.0, 0.0, 0.04);
            pid.enableContinuousInput(-Math.PI, Math.PI);
            return pid;
        }

        public static PIDController makeAlignDrivePID() {
            return new PIDController(5.0, 0.0, 0.01);
        }

        public static PIDController makeBargeAlignDrivePID() {
            return new PIDController(6.0, 0.0, 0.0);
        }
    }

    // Additional constants would need to be added here...
}
