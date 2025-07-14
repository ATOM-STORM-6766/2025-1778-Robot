package org.chillout1778.subsystems;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.chillout1778.Constants;
import org.chillout1778.LogManager;
import org.chillout1778.Robot;

/** 扩展 Phoenix 6 SwerveDrivetrain 类并实现 Subsystem 接口的类， 使其可以在基于命令的项目中轻松使用。 集成了原 Swerve.java 的业务逻辑。 */
public class SwerveNext extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  private static SwerveNext instance;

  public static SwerveNext getInstance() {
    if (instance == null) {
      instance =
          new SwerveNext(
              Constants.SwerveNext.DrivetrainConstants,
              Constants.SwerveNext.FrontLeft,
              Constants.SwerveNext.FrontRight,
              Constants.SwerveNext.BackLeft,
              Constants.SwerveNext.BackRight);
    }
    return instance;
  }

  private boolean isAligned = false;
  private final PIDController xController = Constants.SwerveDriveKinematics.makeAutoDrivePID();
  private final PIDController yController = Constants.SwerveDriveKinematics.makeAutoDrivePID();
  private final PIDController headingController = Constants.SwerveDriveKinematics.makeAutoTurnPID();
  private final boolean[] probablyScoredPoses = new boolean[24 * 4];

  private static final double kSimLoopPeriod = 0.005; // 5毫秒
  private volatile Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean m_hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(Math.PI / 6).per(Second),
              Volts.of(Math.PI),
              null,
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

  public SwerveNext(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Use LogManager to register PID controllers
    LogManager.registerPIDController("Swerve X Controller", xController);
    LogManager.registerPIDController("Swerve Y Controller", yController);
    LogManager.registerPIDController("Swerve Heading Controller", headingController);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public void setNeutralMode(NeutralModeValue mode) {
    for (var module : getModules()) {
      module.getDriveMotor().setNeutralMode(mode);
      module.getSteerMotor().setNeutralMode(mode);
    }
  }

  public Pose2d getEstimatedPose() {
    return this.getState().Pose;
  }

  public void setEstimatedPose(Pose2d pose) {
    this.resetPose(pose);
  }

  public void stop() {
    this.setControl(new SwerveRequest.RobotCentric());
  }

  public boolean getIsAligned() {
    return isAligned;
  }

  public void setIsAligned(boolean aligned) {
    isAligned = aligned;
  }

  public void followPose(Pose2d goalPose) {
    Pose2d currentPose = getEstimatedPose();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), goalPose.getX()),
            yController.calculate(currentPose.getY(), goalPose.getY()),
            headingController.calculate(
                currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));

    this.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public void followSample(SwerveSample sample) {
    Pose2d pose = getEstimatedPose();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));
    this.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public boolean getWithinTolerance(Translation2d t) {
    return getEstimatedPose().getTranslation().getDistance(t)
        < Constants.SwerveDriveKinematics.ALIGNMENT_TOLERANCE;
  }

  public double score(Pose2d p) {
    double translation = p.getTranslation().getDistance(getEstimatedPose().getTranslation());
    double rotation =
        Math.abs(p.getRotation().minus(getEstimatedPose().getRotation()).getRadians());
    return Constants.SwerveDriveKinematics.ALIGN_TRANSLATION_WEIGHT * translation
        + Constants.SwerveDriveKinematics.ALIGN_ANGLE_WEIGHT * rotation;
  }

  public void markPoseScored() {
    getClosestFudgedScoringPose()
        .ifPresent(
            indexedPose -> {
              int idx = indexedPose.getIndex();
              // Assuming Superstructure is also refactored or accessible
              // Superstructure.getInstance().getInputs().wantedScoringLevel.ordinal()
              // For now, using a placeholder or assuming a default level
              probablyScoredPoses[idx * 4] = true;
            });
  }

  public boolean wasPoseScored(int idx) {
    // Assuming Superstructure is also refactored or accessible
    // Superstructure.getInstance().getInputs().wantedScoringLevel.ordinal()
    return probablyScoredPoses[idx * 4];
  }

  public static class IndexedPose2d {
    private final int index;
    private final Pose2d pose;

    public IndexedPose2d(int index, Pose2d pose) {
      this.index = index;
      this.pose = pose;
    }

    public int getIndex() {
      return index;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  public Optional<IndexedPose2d> getClosestFudgedScoringPose() {
    List<Pose2d> poses =
        Robot.getInstance().isRedAlliance()
            ? Constants.Field.redScoringPoses
            : Constants.Field.blueScoringPoses;
    return IntStream.range(0, poses.size())
        .mapToObj(i -> new IndexedPose2d(i, poses.get(i)))
        .filter(
            indexedPose ->
                indexedPose
                        .getPose()
                        .getTranslation()
                        .getDistance(getEstimatedPose().getTranslation())
                    < Constants.SwerveDriveKinematics.MAX_NODE_DISTANCE)
        .min(
            Comparator.comparingDouble(
                indexedPose -> {
                  double fudge =
                      wasPoseScored(indexedPose.getIndex())
                          ? Constants.SwerveDriveKinematics.ALREADY_SCORED_BADNESS
                          : 0.0;
                  return fudge + score(indexedPose.getPose());
                }));
  }

  public Optional<Pose2d> getClosestAlgaeGrabPose() {
    List<Pose2d> poses =
        Robot.getInstance().isRedAlliance()
            ? Constants.Field.redAlgaeGrabbingPose
            : Constants.Field.blueAlgaeGrabbingPose;
    return poses.stream()
        .filter(
            p ->
                p.getTranslation().getDistance(getEstimatedPose().getTranslation())
                    < Constants.SwerveDriveKinematics.MAX_NODE_DISTANCE)
        .min(Comparator.comparingDouble(this::score));
  }

  public Optional<Pose2d> getClosestTroughScoringPose() {
    List<Pose2d> poses =
        Robot.getInstance().isRedAlliance()
            ? Constants.Field.redTroughScoringPoses
            : Constants.Field.blueTroughScoringPoses;
    return poses.stream()
        .filter(
            p ->
                p.getTranslation().getDistance(getEstimatedPose().getTranslation())
                    < Constants.SwerveDriveKinematics.MAX_NODE_DISTANCE)
        .min(Comparator.comparingDouble(this::score));
  }

  private static final double BARGE_SCORING_POSITION_TOLERANCE = 0.1;

  public boolean atGoodScoringDistance() {
    double x = getEstimatedPose().getX();
    if (!Robot.getInstance().isOnRedSide()) {
      return x > Constants.Field.BLUE_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
          && x < Constants.Field.BLUE_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE;
    } else {
      return x > Constants.Field.RED_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
          && x < Constants.Field.RED_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE;
    }
  }

  @Override
  public void periodic() {
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
      m_simNotifier =
          new Notifier(
              () -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
              });
      m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
  }
}
