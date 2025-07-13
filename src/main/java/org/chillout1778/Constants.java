package org.chillout1778;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

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
    public static final int ADDRESSABLE_LED = 0;
    public static final int ARM_ABSOLUTE_ENCODER = 1;
    public static final int INTAKE_LINEBREAK = 2;
    public static final int INTAKE_LINEBREAK_FOLLOW = 3;
    public static final int DISABLE_BREAK_MODE = 4;
  }

  public static class Lights {
    // right - 34 leds, left - 33, cross - 20
    // LED segment lengths
    public static final int TOTAL_LENGTH = 86;

    // LED segment indices
    public static final int RIGHT_SEGMENT_START = 0;
    public static final int RIGHT_SEGMENT_END = 32;
    public static final int CROSS_SEGMENT_START = 33;
    public static final int CROSS_SEGMENT_END = 52;
    public static final int LEFT_SEGMENT_START = 53;
    public static final int LEFT_SEGMENT_END = 85;

    // Battery charge progress bar indices
    public static final int BATTERY_PROGRESS_START = 34;
    public static final int BATTERY_PROGRESS_END = 48;
    public static final int BATTERY_PROGRESS_REVERSE_START = 49;
    public static final int BATTERY_PROGRESS_REVERSE_END = 53;
  }

  public static class Vision {
    public static final AprilTagFields FIELD_TYPE = AprilTagFields.k2025ReefscapeWelded;

    public static final String FRONT_RIGHT_NAME = "Front Right";
    public static final String FRONT_LEFT_NAME = "Front Left";
    public static final String BACK_RIGHT_NAME = "Back Right";
    public static final String BACK_LEFT_NAME = "Back Left";

    // positive x is to the front (intake side), positive y is to the left, and
    // counterclockwise (when looking down at the robot) is positive theta
    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.016429, -0.3234519, 0.187350),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-60.0)));
    public static final Transform3d BACK_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.079749, -0.324875, 0.187354),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-(180.0 - 60.0))));
    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(0.016429, 0.324519, 0.18735),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(60.0)));
    public static final Transform3d BACK_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(0.079749, 0.324875, 0.187354),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(180.0 - 60.0)));
  }

  public static class Field {
    public static final double FIELD_X_SIZE = 17.548249;
    public static final double FIELD_Y_SIZE = 8.051800;

    public static final double BLUE_BARGE_SCORING_X = 7.6;
    public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;

    public static final double SAFE_WALL_DISTANCE = 1.0;

    /** Coordinates of the center of the reef on the blue side of the field */
    public static final Translation2d BLUE_REEF_CENTER =
        new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

    /**
     * Distance between the center of the robot and the center of the reef when the robot is at the
     * center of one of the reef's edges.
     */
    public static final double ROBOT_REEF_CENTER_DISTANCE = Units.inchesToMeters(32.25 + 18.0);

    /** Branch offset from the center of one of the reef's edges */
    public static final double REEF_BRANCH_OFFSET_DISTANCE = Units.inchesToMeters(12.9375 / 2);

    public static final double TROUGH_OFFSET_DISTANCE = Units.inchesToMeters(14.5 / 2);

    public static final List<Pose2d> blueScoringPoses;
    public static final List<Pose2d> blueTroughScoringPoses;
    public static final List<Pose2d> blueAlgaeGrabbingPose;
    public static final List<Pose2d> redScoringPoses;
    public static final List<Pose2d> redTroughScoringPoses;
    public static final List<Pose2d> redAlgaeGrabbingPose;

    static {
      blueScoringPoses =
          IntStream.range(0, 360)
              .filter(angle -> angle % 60 == 0)
              .boxed()
              .flatMap(
                  angle -> {
                    return IntStream.of(1, -1)
                        .boxed()
                        .flatMap(
                            direction -> {
                              return IntStream.of(1, -1)
                                  .mapToObj(
                                      side -> {
                                        return new Pose2d(BLUE_REEF_CENTER, new Rotation2d(0.0))
                                            .plus(
                                                new Transform2d(
                                                    new Translation2d(
                                                        -ROBOT_REEF_CENTER_DISTANCE,
                                                        Rotation2d.fromDegrees(angle)),
                                                    new Rotation2d()))
                                            .plus(
                                                new Transform2d(
                                                    new Translation2d(
                                                        direction * REEF_BRANCH_OFFSET_DISTANCE,
                                                        Rotation2d.fromDegrees(angle + 90.0)),
                                                    Rotation2d.fromDegrees(angle)))
                                            .plus(
                                                new Transform2d(
                                                    new Translation2d(
                                                        0.0, side * Arm.CORAL_CENTER_OFFSET),
                                                    new Rotation2d(-side * Math.PI / 2)));
                                      });
                            });
                  })
              .collect(Collectors.toList());

      blueTroughScoringPoses =
          IntStream.range(0, 360)
              .filter(angle -> angle % 60 == 0)
              .boxed()
              .flatMap(
                  angle -> {
                    return IntStream.of(1, -1)
                        .mapToObj(
                            direction -> {
                              return new Pose2d(BLUE_REEF_CENTER, new Rotation2d(0.0))
                                  .plus(
                                      new Transform2d(
                                          new Translation2d(
                                              -ROBOT_REEF_CENTER_DISTANCE,
                                              Rotation2d.fromDegrees(angle)),
                                          new Rotation2d()))
                                  .plus(
                                      new Transform2d(
                                          new Translation2d(
                                              direction * TROUGH_OFFSET_DISTANCE,
                                              Rotation2d.fromDegrees(angle + 90.0)),
                                          Rotation2d.fromDegrees(angle)));
                            });
                  })
              .collect(Collectors.toList());

      blueAlgaeGrabbingPose =
          IntStream.range(0, 360)
              .filter(angle -> angle % 60 == 0)
              .boxed()
              .flatMap(
                  angle -> {
                    return IntStream.of(1, -1)
                        .mapToObj(
                            side -> {
                              return new Pose2d(BLUE_REEF_CENTER, new Rotation2d())
                                  .plus(
                                      new Transform2d(
                                          new Translation2d(
                                              -ROBOT_REEF_CENTER_DISTANCE + .05,
                                              Rotation2d.fromDegrees(angle)),
                                          Rotation2d.fromDegrees(angle)))
                                  .plus(
                                      new Transform2d(
                                          new Translation2d(0.0, side * Arm.CORAL_CENTER_OFFSET),
                                          new Rotation2d(-side * Math.PI / 2)));
                            });
                  })
              .collect(Collectors.toList());

      redScoringPoses = blueScoringPoses.stream().map(Utils::mirror).collect(Collectors.toList());
      redTroughScoringPoses =
          blueTroughScoringPoses.stream().map(Utils::mirror).collect(Collectors.toList());
      redAlgaeGrabbingPose =
          blueAlgaeGrabbingPose.stream().map(Utils::mirror).collect(Collectors.toList());
    }
  }

  public static class Intake {
    public static final double ZERO_MIN_CURRENT = 20.0; // amps
    public static final double SETPOINT_THRESHOLD = Math.toRadians(7.0);
    public static final double ZERO_VOLTAGE = -0.7;
    public static final double ULTRASONIC_SENSOR_THRESHOLD = 0.02;

    private static final double GEAR_RATIO =
        1.0 / ((12.0 / 40.0) * (18.0 / 46.0) * (18.0 / 60.0) * (12.0 / 32.0));

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
      config.MotionMagic.MotionMagicAcceleration = 100.0; // reduced from 200.0 to 100.0
      config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // reduced from 2.0 to 1.0
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      return config;
    }

    public static TalonFXConfiguration getRollerConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.CurrentLimits.StatorCurrentLimit = 80.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      return config;
    }

    public static TalonFXConfiguration getCenteringConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      return config;
    }
  }

  public static class Elevator {
    public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
    public static final double GEAR_RATIO = 4.0;
    public static final double ZERO_VOLTAGE = -0.05;
    public static final double ZERO_MIN_CURRENT = 0.1; // amps
    public static final double SETPOINT_THRESHOLD = 0.01;
    public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;
    public static final double COLLISION_AVOIDANCE_MARGIN = 1.0;
    public static final double MAX_EXTENSION = Units.inchesToMeters(55.0);
    public static final double SAFE_HEIGHT = 0.837198 - 0.01;

    public static TalonFXConfiguration getMotorConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Feedback.SensorToMechanismRatio = GEAR_RATIO / (SPOOL_RADIUS * 2 * Math.PI);
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.Slot0.kS = 0.0;
      config.Slot0.kV = 0.0;
      config.Slot0.kA = 0.0;
      config.Slot0.kG = 0.37;
      config.Slot0.kP = 70.0;
      config.MotionMagic.MotionMagicAcceleration = 7.0; // reduced from 14.0 to 10.0
      config.MotionMagic.MotionMagicCruiseVelocity = 1.5; // reduced from 3.0 to 2.5
      return config;
    }

    public static TalonFXConfiguration getFollowerConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      return config;
    }
  }

  public static class Arm {
    public static final double POSITION_DEPENDENT_KG = 0.33;
    public static final double ALLOWED_OPERATING_RANGE_MIN = Math.toRadians(-350.0);
    public static final double ALLOWED_OPERATING_RANGE_MAX = Math.toRadians(350.0);
    public static final double PIVOT_ENCODER_RATIO =
        (36.0 / 16.0) * (36.0 / 16.0) * (60.0 / 24.0) * (12.0 / 54.0);
    public static final double PIVOT_GEAR_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (12.0 / 54.0);
    public static final double PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS = 0.431;
    public static final double CORAL_CENTER_OFFSET = Units.inchesToMeters(9.5);
    public static final double SAFE_DISTANCE_FROM_REEF_CENTER = Units.inchesToMeters(70.0);
    public static final double SAFE_PLACEMENT_DISTANCE = Units.inchesToMeters(60.0);
    public static final double SAFE_BARGE_DISTANCE = Units.inchesToMeters(50.0);
    public static final double SAFE_INSIDE_ROBOT_ANGLE = Math.toRadians(40.0);
    public static final double SETPOINT_THRESHOLD = 0.1;
    public static final double CURRENT_DRAW = 20.0;
    public static final double IDLE_CURRENT_DRAW = 10.0;

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
      config.MotionMagic.MotionMagicAcceleration = 2.0; // reduced from 4.5 to 3.0
      config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // reduced from 2.0 to 1.5 rps
      config.CurrentLimits.StatorCurrentLimit = 70.0;
      config.CurrentLimits.SupplyCurrentLimit = 50.0;
      return config;
    }

    public static TalonFXConfiguration getRollerConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.CurrentLimits.StatorCurrentLimit = 80.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      return config;
    }
  }

  public class SwerveNext {

    public static final PhoenixPIDController HeadingController;

    static {
      HeadingController = new PhoenixPIDController(3, 0, 0.01);
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // 两组增益都需要根据您的具体机器人进行调整

    // 转向电机使用任何 SwerveModule.SteerRequestType 控制请求，
    // 输出类型由 SwerveModuleConstants.SteerMotorClosedLoopOutput 指定
    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0.01)
            .withKS(0.25727)
            .withKV(1.6302)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // 当使用闭环控制时，驱动电机使用由 SwerveModuleConstants.DriveMotorClosedLoopOutput 指定的控制输出类型
    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(0.17494)
            .withKI(0)
            .withKD(0)
            .withKS(0.1237)
            .withKV(0.12354)
            .withKA(0.014822);

    // 用于转向电机的闭环输出类型；
    // 这会影响转向电机的 PID/FF 增益
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // 用于驱动电机的闭环输出类型；
    // 这会影响驱动电机的 PID/FF 增益
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // 驱动电机使用的电机类型
    private static final DriveMotorArrangement kDriveMotorType =
        DriveMotorArrangement.TalonFX_Integrated;
    // 转向电机使用的电机类型
    private static final SteerMotorArrangement kSteerMotorType =
        SteerMotorArrangement.TalonFX_Integrated;

    // 用于转向电机的远程传感器反馈类型；
    // 当没有 Pro 许可证时，FusedCANcoder/SyncCANcoder 自动降级为 RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // 车轮开始打滑时的定子电流；
    // 这需要根据您的具体机器人进行调整
    private static final Current kSlipCurrent = Amps.of(70.0);

    // 驱动和转向电机以及方位编码器的初始配置；这些不能为空。
    // 某些配置将被覆盖；请查看 `with*InitialConfigs()` API 文档。
    private static final TalonFXConfiguration driveInitialConfigs =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(false).withBeepOnConfig(false));
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withAudio(new AudioConfigs().withBeepOnBoot(false).withBeepOnConfig(false))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve 方位不需要太大的扭矩输出，所以我们可以设置一个相对较低的
                    // 定子电流限制，以帮助避免断电而不影响性能。
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Pigeon 2 的配置；如果不需要应用 Pigeon 2 配置，则保持为 null
    private static final Pigeon2Configuration pigeonConfigs = null;

    // 设备所在的 CAN 总线；
    // 所有 swerve 设备必须共享同一个 CAN 总线
    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

    // 在 12V 输出电压下的理论最大速度（m/s）；
    // 这需要根据您的具体机器人进行调整
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(0.5);

    // 方位每旋转 1 圈会导致驱动电机转动 kCoupleRatio 圈；
    // 这可能需要根据您的具体机器人进行调整
    private static final double kCoupleRatio = 50.0 / 14; // 值为L3的第一级减速比 14：50 的倒数

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = true;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 30;

    // 这些仅用于仿真
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // 克服摩擦所需的模拟电压
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 9;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.140380859375);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(13.393747);
    private static final Distance kFrontLeftYPos = Inches.of(13.393747);

    // Back Left
    private static final int kBackLeftDriveMotorId = 7;
    private static final int kBackLeftSteerMotorId = 8;
    private static final int kBackLeftEncoderId = 11;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.314208984375);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-13.393747);
    private static final Distance kBackLeftYPos = Inches.of(13.393747);

    // Back Right
    private static final int kBackRightDriveMotorId = 5;
    private static final int kBackRightSteerMotorId = 6;
    private static final int kBackRightEncoderId = 12;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.15283203125);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-13.393747);
    private static final Distance kBackRightYPos = Inches.of(-13.393747);

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 10;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.236083984375);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(13.393747);
    private static final Distance kFrontRightYPos = Inches.of(-13.393747);

    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightEncoderInverted);
  }

  public static class SwerveDriveKinematics {
    public static final double ALIGNMENT_TOLERANCE = 0.04;
    public static final double STARTING_TOLERANCE = 0.15;

    // How far the swerve modules are from (0,0).
    public static final double XY_DISTANCE = Units.inchesToMeters(13.393747);

    // How fast the robot can move in a straight line (meters/sec).
    public static final double MAX_VELOCITY = 0.5;

    // How fast the robot can rotate (radians/sec).
    public static final double MAX_ANGULAR_VELOCITY = 10;

    // Alignment constants
    public static final double maxAlignTranslationSpeed = 1.5;
    public static final double maxAlignRotationSpeed = 2.5;
    public static final double maxBargeAlignTranslationSpeed = 1.5;
    public static final double maxBargeAlignRotationSpeed = 1.5;
    public static final double MAX_NODE_DISTANCE = 3.0; // meters

    public static final double ALIGN_ANGLE_WEIGHT = 2.7;
    public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
    public static final double ALREADY_SCORED_BADNESS =
        0.5 + Units.inchesToMeters(12.9375) * ALIGN_TRANSLATION_WEIGHT * (1 - 2 * 0.3);

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

  public static final List<Map.Entry<Double, Double>> armElevatorPairs =
      List.of(
          Map.entry(Math.toRadians(7.0), 0.0),
          Map.entry(Math.toRadians(18.775917), 0.044542),
          Map.entry(Math.toRadians(26.241907), 0.068086),
          Map.entry(Math.toRadians(34.899764), 0.100567),
          Map.entry(Math.toRadians(44.171701), 0.137335),
          Map.entry(Math.toRadians(55.042173), 0.184139),
          Map.entry(Math.toRadians(68.556032), 0.253736),
          Map.entry(Math.toRadians(180.0 - 96.880), 0.324134),
          Map.entry(Math.toRadians(180.0 - 87.206214), 0.371332),
          Map.entry(Math.toRadians(180.0 - 82.416832), 0.396130),
          Map.entry(Math.toRadians(180.0 - 71.851303), 0.441318),
          Map.entry(Math.toRadians(180.0 - 68.941494), 0.483176),
          Map.entry(Math.toRadians(180.0 - 65.996990), 0.521174),
          Map.entry(Math.toRadians(180.0 - 64.161654), 0.558276),
          Map.entry(Math.toRadians(180.0 - 62.064537), 0.619367),
          Map.entry(Math.toRadians(180.0 - 58.290246), 0.654566),
          Map.entry(Math.toRadians(180.0 - 55.419787), 0.701323 - .0254),
          Map.entry(Math.toRadians(180.0 - 47.462013), 0.750120 - .0254),
          Map.entry(Math.toRadians(180.0 - 40.585547), 0.802161 - .0254),
          Map.entry(Math.toRadians(180.0), Elevator.SAFE_HEIGHT));

  public static final List<Map.Entry<Double, Double>> armInterpolationIntakeDown =
      List.of(
          Map.entry(Math.toRadians(88.0), 0.0),
          Map.entry(Math.toRadians(103.296), 0.12),
          Map.entry(Math.toRadians(113.75), 0.202),
          Map.entry(Math.toRadians(117.77), 0.333),
          Map.entry(Math.toRadians(131.77), 0.456),
          Map.entry(Math.toRadians(137.593), 0.533),
          Map.entry(Math.toRadians(180.0), 0.660));
}
