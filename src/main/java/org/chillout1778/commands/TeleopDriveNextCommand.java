package org.chillout1778.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.chillout1778.Constants;
import org.chillout1778.Controls;
import org.chillout1778.LogManager;
import org.chillout1778.Robot;
import org.chillout1778.Utils;
import org.chillout1778.subsystems.SwerveNext;

public class TeleopDriveNextCommand extends Command {
  private final SwerveNext swerve = SwerveNext.getInstance();
  private final Supplier<Controls.DriveInputs> driveInputsSupplier;

  private Controls.AlignMode previousAlignMode = Controls.AlignMode.None;

  // PID controllers for alignment
  private final PIDController xPID = Constants.SwerveDriveKinematics.makeAlignDrivePID();
  private final PIDController yPID = Constants.SwerveDriveKinematics.makeAlignDrivePID();
  private final PIDController turnPID = Constants.SwerveDriveKinematics.makeAlignTurnPID();

  private Pose2d lastTargetPose = null;

  public TeleopDriveNextCommand(Supplier<Controls.DriveInputs> driveInputsSupplier) {
    this.driveInputsSupplier = driveInputsSupplier;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    Controls.DriveInputs inputs = driveInputsSupplier.get();
    if (Robot.getInstance().isRedAlliance()) inputs = inputs.redFlipped();

    if (inputs.isNonZero()) {
      inputs =
          new Controls.DriveInputs(
              inputs.getForward(),
              inputs.getLeft(),
              inputs.getRotation(),
              inputs.getDeadzone(),
              Controls.AlignMode.None); // if drive inputs non-zero, don't do align
    }
    if (inputs.getAlignMode() != previousAlignMode) {
      xPID.reset();
      yPID.reset();
      turnPID.reset();
    }
    previousAlignMode = inputs.getAlignMode();

    swerve.setIsAligned(false); // default to false, override later

    ChassisSpeeds speeds;
    if (inputs.getAlignMode() == Controls.AlignMode.None) {
      speeds = chassisSpeedsFromDriveInputs(inputs);
    } else if (inputs.getAlignMode() == Controls.AlignMode.BargeAlign) {
      speeds =
          new ChassisSpeeds(
              fixBargeTranslationInput(
                  xPID.calculate(
                      swerve.getEstimatedPose().getX(),
                      Robot.getInstance().isOnRedSide()
                          ? Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X
                          : Constants.Field.BLUE_BARGE_SCORING_X)),
              0.0,
              fixRotationInput(
                  turnPID.calculate(
                      swerve.getEstimatedPose().getRotation().getRadians(),
                      Math.abs(Math.PI / 2 - swerve.getEstimatedPose().getRotation().getRadians())
                              < Math.abs(
                                  -Math.PI / 2
                                      - swerve.getEstimatedPose().getRotation().getRadians())
                          ? Math.PI / 2
                          : -Math.PI / 2)));

      lastTargetPose =
          new Pose2d(
              Robot.getInstance().isOnRedSide()
                  ? Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X
                  : Constants.Field.BLUE_BARGE_SCORING_X,
              swerve.getEstimatedPose().getY(),
              new Rotation2d(
                  Math.abs(Math.PI / 2 - swerve.getEstimatedPose().getRotation().getRadians())
                          < Math.abs(
                              -Math.PI / 2 - swerve.getEstimatedPose().getRotation().getRadians())
                      ? Math.PI / 2
                      : -Math.PI / 2));
    } else {
      Pose2d pose = null;
      switch (inputs.getAlignMode()) {
        case TroughAlign:
          pose = swerve.getClosestTroughScoringPose().orElse(null);
          lastTargetPose = pose;
          break;
        case ReefAlign:
          pose =
              swerve
                  .getClosestFudgedScoringPose()
                  .map(SwerveNext.IndexedPose2d::getPose)
                  .orElse(null);
          lastTargetPose = pose;
          break;
        case AlgaeAlign:
          pose = swerve.getClosestAlgaeGrabPose().orElse(null);
          lastTargetPose = pose;
          break;
        default:
          break;
      }

      if (pose == null) {
        speeds = chassisSpeedsFromDriveInputs(inputs);
      } else {
        swerve.setIsAligned(swerve.getWithinTolerance(pose.getTranslation()));
        speeds =
            new ChassisSpeeds(
                fixTranslationInput(xPID.calculate(swerve.getEstimatedPose().getX(), pose.getX())),
                fixTranslationInput(yPID.calculate(swerve.getEstimatedPose().getY(), pose.getY())),
                fixRotationInput(
                    turnPID.calculate(
                        swerve.getEstimatedPose().getRotation().getRadians(),
                        pose.getRotation().getRadians())));
      }
    }

    swerve.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  private double fixRotationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.SwerveDriveKinematics.maxAlignRotationSpeed,
            Math.min(Constants.SwerveDriveKinematics.maxAlignRotationSpeed, n)),
        0.03);
  }

  private double fixTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.SwerveDriveKinematics.maxAlignTranslationSpeed,
            Math.min(Constants.SwerveDriveKinematics.maxAlignTranslationSpeed, n)),
        0.03);
  }

  private double fixBargeTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.SwerveDriveKinematics.maxBargeAlignTranslationSpeed,
            Math.min(Constants.SwerveDriveKinematics.maxBargeAlignTranslationSpeed, n)),
        0.03);
  }

  private ChassisSpeeds chassisSpeedsFromDriveInputs(Controls.DriveInputs inputs) {
    double x = inputs.getForward();
    double y = inputs.getLeft();
    double rotation = inputs.getRotation();

    double theta = Math.atan2(y, x);
    double r = Math.hypot(x, y);
    r = Utils.deadZone(r, inputs.getDeadzone());
    rotation = Utils.deadZone(rotation, inputs.getDeadzone());
    r = r * r;
    rotation = rotation * rotation * Math.signum(rotation);

    double actualX = r * Math.cos(theta) * Constants.SwerveDriveKinematics.MAX_VELOCITY;
    double actualY = r * Math.sin(theta) * Constants.SwerveDriveKinematics.MAX_VELOCITY;
    double actualRotation = rotation * Constants.SwerveDriveKinematics.MAX_ANGULAR_VELOCITY;

    return new ChassisSpeeds(actualX, actualY, actualRotation);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TeleopDriveNextCommand");
    builder.addStringProperty(
        "Align Mode", () -> driveInputsSupplier.get().getAlignMode().toString(), null);
    builder.addBooleanProperty("Is Aligned", () -> swerve.getIsAligned(), null);
    builder.addDoubleArrayProperty(
        "Target Pose",
        () -> {
          if (lastTargetPose != null) {
            return new double[] {
              lastTargetPose.getX(),
              lastTargetPose.getY(),
              lastTargetPose.getRotation().getRadians()
            };
          }
          return new double[] {};
        },
        null);

    // Use LogManager to register PID controllers
    LogManager.registerPIDController("Teleop Align X PID", xPID);
    LogManager.registerPIDController("Teleop Align Y PID", yPID);
    LogManager.registerPIDController("Teleop Align Turn PID", turnPID);
  }
}
