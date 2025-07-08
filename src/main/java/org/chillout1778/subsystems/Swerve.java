package org.chillout1778.subsystems;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import org.chillout1778.Constants;
import org.chillout1778.Robot;

public class Swerve extends SubsystemBase {
  private static Swerve instance;

  public static Swerve getInstance() {
    // if (instance == null) {
    //   instance = new Swerve();
    // }
    // return instance;
    return null;
  }

  private final Pigeon2 gyro;
  private final SwerveModule[] modules;
  private final Field2d fieldEstimate = new Field2d();
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  public boolean isAligned = false;

  private final PIDController xController = new PIDController(12.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(12.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(5.0, 0.0, 0.0);

  private final boolean[] probablyScoredPoses = new boolean[24 * 4];

  private Swerve() {
    gyro = new Pigeon2(Constants.CanIds.GYRO);
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    modules =
        new SwerveModule[] {
          new SwerveModule(
              "Front Left",
              Constants.CanIds.SWERVE_FRONT_LEFT_DRIVE,
              Constants.CanIds.SWERVE_FRONT_LEFT_TURN,
              Constants.CanIds.SWERVE_FRONT_LEFT_CANCODER,
              Constants.Swerve.FRONT_LEFT_ENCODER_OFFSET,
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new SwerveModule(
              "Front Right",
              Constants.CanIds.SWERVE_FRONT_RIGHT_DRIVE,
              Constants.CanIds.SWERVE_FRONT_RIGHT_TURN,
              Constants.CanIds.SWERVE_FRONT_RIGHT_CANCODER,
              Constants.Swerve.FRONT_RIGHT_ENCODER_OFFSET,
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new SwerveModule(
              "Back Left",
              Constants.CanIds.SWERVE_BACK_LEFT_DRIVE,
              Constants.CanIds.SWERVE_BACK_LEFT_TURN,
              Constants.CanIds.SWERVE_BACK_LEFT_CANCODER,
              Constants.Swerve.BACK_LEFT_ENCODER_OFFSET,
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new SwerveModule(
              "Back Right",
              Constants.CanIds.SWERVE_BACK_RIGHT_DRIVE,
              Constants.CanIds.SWERVE_BACK_RIGHT_TURN,
              Constants.CanIds.SWERVE_BACK_RIGHT_CANCODER,
              Constants.Swerve.BACK_RIGHT_ENCODER_OFFSET,
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive)
        };

    for (SwerveModule module : modules) {
      Shuffleboard.getTab("Swerve").add(module.getName(), module);
    }
    Shuffleboard.getTab("Swerve").add("swerve actual object", this);
    Shuffleboard.getTab("Swerve").add("Swerve Estimated Pose Field", fieldEstimate);

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // FL
            new Translation2d(Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // FR
            new Translation2d(-Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // BL
            new Translation2d(-Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE) // BR
            );

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(getGyroAngle()),
            getModulePositions(),
            new Pose2d(0.0, 0.0, new Rotation2d(getGyroAngle())),
            VecBuilder.fill(0.1, 0.1, 0.1), // odometry
            VecBuilder.fill(0.9, 0.9, 2.0) // vision
            );

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public boolean getWithinTolerance(Translation2d t) {
    return getEstimatedPose().getTranslation().getDistance(t)
        < Constants.Swerve.ALIGNMENT_TOLERANCE;
  }

  public double getGyroAngle() {
    return MathUtil.angleModulus(gyro.getRotation2d().getRadians());
  }

  public void setGyroAngle(double desiredAngle) {
    gyro.setYaw(MathUtil.angleModulus(desiredAngle) / Math.PI * 180);
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getEstimatedPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.kDefaultPeriod);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    for (int i = 0; i < modules.length; i++) {
      modules[i].driveState(moduleStates[i]);
    }
    poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
  }

  public void stop() {
    driveRobotRelative(new ChassisSpeeds());
  }

  public void followSample(SwerveSample sample) {
    Pose2d pose = getEstimatedPose();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));
    driveFieldRelative(speeds);
  }

  public void followPose(Pose2d goalPose) {
    Pose2d currentPose = getEstimatedPose();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), goalPose.getX()),
            yController.calculate(currentPose.getY(), goalPose.getY()),
            headingController.calculate(
                currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
    driveFieldRelative(speeds);
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setEstimatedPose(Pose2d p) {
    poseEstimator.resetPosition(new Rotation2d(getGyroAngle()), getModulePositions(), p);
  }

  public double score(Pose2d p) {
    double translation = p.getTranslation().getDistance(getEstimatedPose().getTranslation());
    double rotation =
        Math.abs(p.getRotation().minus(getEstimatedPose().getRotation()).getRadians());
    return Constants.Swerve.ALIGN_TRANSLATION_WEIGHT * translation
        + Constants.Swerve.ALIGN_ANGLE_WEIGHT * rotation;
  }

  public void markPoseScored() {
    getClosestFudgedScoringPose()
        .ifPresent(
            indexedPose -> {
              int idx = indexedPose.getIndex();
              probablyScoredPoses[
                      idx * 4
                          + Superstructure.getInstance().getInputs().wantedScoringLevel.ordinal()] =
                  true;
            });
  }

  public boolean wasPoseScored(int idx) {
    return probablyScoredPoses[
        idx * 4 + Superstructure.getInstance().getInputs().wantedScoringLevel.ordinal()];
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
                    < Constants.Swerve.MAX_NODE_DISTANCE)
        .min(
            Comparator.comparingDouble(
                indexedPose -> {
                  double fudge =
                      wasPoseScored(indexedPose.getIndex())
                          ? Constants.Swerve.ALREADY_SCORED_BADNESS
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
                    < Constants.Swerve.MAX_NODE_DISTANCE)
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
                    < Constants.Swerve.MAX_NODE_DISTANCE)
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
    Vision.getInstance().periodicAddMeasurements(poseEstimator);
    poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
    fieldEstimate.setRobotPose(getEstimatedPose());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("gyro angle", () -> Math.toDegrees(getGyroAngle()), null);
    builder.addDoubleProperty(
        "vision angle", () -> getEstimatedPose().getRotation().getDegrees(), null);
    builder.addBooleanProperty("is red?", () -> Robot.getInstance().isRedAlliance(), null);
    builder.addDoubleProperty(
        "raw (not wrapped) gyro yaw", () -> gyro.getRotation2d().getRadians(), null);
    builder.addStringProperty("odometry pose", () -> getEstimatedPose().toString(), null);
  }
}
