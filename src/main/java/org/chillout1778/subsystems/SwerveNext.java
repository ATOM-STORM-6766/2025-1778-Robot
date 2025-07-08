package org.chillout1778.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.chillout1778.Constants;
import org.chillout1778.Robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * 扩展 Phoenix 6 SwerveDrivetrain 类并实现 Subsystem 接口的类，
 * 使其可以在基于命令的项目中轻松使用。
 */
public class SwerveNext extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static SwerveNext instance;

    public static SwerveNext getInstance() {
        if (instance == null) {
            instance = new SwerveNext(Constants.SwerveNext.DrivetrainConstants,
                    Constants.SwerveNext.FrontLeft, Constants.SwerveNext.FrontRight, Constants.SwerveNext.BackLeft,
                    Constants.SwerveNext.BackRight);
        }
        return instance;
    }

    private static final double kSimLoopPeriod = 0.005; // 5毫秒
    private volatile Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** 在机器人中心路径跟踪期间应用的转向请求 */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    /* 在SysId特性化过程中应用的转向请求 */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* 用于特性化平移的SysId程序。这用于找到驱动电机的PID增益。 */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // 使用默认斜率（1 V/s）
                    Volts.of(4), // 将动态步进电压降至4V以防止断电
                    null, // 使用默认超时（10秒）
                    // 使用SignalLogger类记录状态
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /* 用于特性化转向的SysId程序。这用于找到转向电机的PID增益。 */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // 使用默认斜率（1 V/s）
                    Volts.of(7), // 使用7V的动态电压
                    null, // 使用默认超时（10秒）
                    // 使用SignalLogger类记录状态
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * 用于特性化旋转的SysId程序。
     * 这用于找到FieldCentricFacingAngle HeadingController的PID增益。
     * 有关将日志导入SysId的信息，请参见SwerveRequest.SysIdSwerveRotation的文档。
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* 这是弧度/秒²，但SysId只支持"伏特/秒" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* 这是弧度/秒，但SysId只支持"伏特" */
                    Volts.of(Math.PI),
                    null, // 使用默认超时（10秒）
                    // 使用SignalLogger类记录状态
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output实际上是弧度/秒，但SysId只支持"伏特" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* 同时记录请求的输出用于SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* 要测试的SysId程序 */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    /**
     * 使用指定的常量构造CTRE SwerveDrivetrain。
     * 
     * 这会构造底层硬件设备，因此用户不应自行构造设备。
     * 如果他们需要这些设备，可以通过类中的getter方法访问它们。
     *
     * @param drivetrainConstants 整个驱动系统的常量
     * @param modules             每个具体模块的常量
     */
    public SwerveNext(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules);
    }

    /**
     * 使用指定的常量构造CTRE SwerveDrivetrain。
     * 
     * 这会构造底层硬件设备，因此用户不应自行构造设备。
     * 如果他们需要这些设备，可以通过类中的getter方法访问它们。
     *
     * @param drivetrainConstants     整个驱动系统的常量
     * @param odometryUpdateFrequency 里程计循环的运行频率。如果未指定或设置为0 Hz，
     *                                在CAN FD上为250 Hz，在CAN 2.0上为100 Hz
     * @param modules                 每个具体模块的常量
     */
    public SwerveNext(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules);
    }

    /**
     * 使用指定的常量构造CTRE SwerveDrivetrain。
     * 
     * 这会构造底层硬件设备，因此用户不应自行构造设备。
     * 如果他们需要这些设备，可以通过类中的getter方法访问它们。
     *
     * @param drivetrainConstants       整个驱动系统的常量
     * @param odometryUpdateFrequency   里程计循环的运行频率。如果未指定或设置为0 Hz，
     *                                  在CAN FD上为250 Hz，在CAN 2.0上为100 Hz
     * @param odometryStandardDeviation 里程计计算的标准差，格式为[x, y, theta]ᵀ，
     *                                  单位为米和弧度
     * @param visionStandardDeviation   视觉计算的标准差，格式为[x, y, theta]ᵀ，
     *                                  单位为米和弧度
     * @param modules                   每个具体模块的常量
     */
    public SwerveNext(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation,
                modules);
    }

    /**
     * 返回一个将指定控制请求应用于此摆动驱动系统的命令。
     *
     * @param request 返回要应用的请求的函数
     * @return 要运行的命令
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * 在给定方向上运行由{@link #m_sysIdRoutineToApply}指定的SysId准静态测试。
     *
     * @param direction SysId准静态测试的方向
     * @return 要运行的命令
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * 在给定方向上运行由{@link #m_sysIdRoutineToApply}指定的SysId动态测试。
     *
     * @param direction SysId动态测试的方向
     * @return 要运行的命令
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * 设置所有模块的空档模式
     * 
     * @param mode 要设置的空档模式
     */
    public void setNeutralMode(NeutralModeValue mode) {
        for (var module : getModules()) {
            module.getDriveMotor().setNeutralMode(mode);
            module.getSteerMotor().setNeutralMode(mode);
        }
    }

    public Pose2d getEstimatedPose() {
        return this.getState().Pose;
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    Robot.getInstance().isRedAlliance()
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (m_simNotifier == null) {
            m_lastSimTime = Utils.getCurrentTimeSeconds();

            /* 以更快的速率运行仿真，使PID增益表现更合理 */
            m_simNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;

                /* 使用测量的时间差，从WPILib获取电池电压 */
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
            m_simNotifier.startPeriodic(kSimLoopPeriod);
        }
    }
}
