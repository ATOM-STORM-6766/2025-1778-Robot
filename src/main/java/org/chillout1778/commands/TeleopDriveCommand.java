package org.chillout1778.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.chillout1778.Constants;
import org.chillout1778.Controls;
import org.chillout1778.Robot;
import org.chillout1778.Utils;
import org.chillout1778.subsystems.Swerve;

public class TeleopDriveCommand extends Command {
  private final Supplier<Controls.DriveInputs> driveInputsSupplier;

  private Controls.AlignMode previousAlignMode = Controls.AlignMode.None;

  // PID controllers for alignment
  private final PIDController xPID = Constants.Swerve.makeAlignDrivePID();
  private final PIDController yPID = Constants.Swerve.makeAlignDrivePID();
  private final PIDController turnPID = Constants.Swerve.makeAlignTurnPID();

  public TeleopDriveCommand(Supplier<Controls.DriveInputs> driveInputsSupplier) {
    this.driveInputsSupplier = driveInputsSupplier;
    addRequirements(Swerve.getInstance());
  }

  @Override
  public void execute() {
    Controls.DriveInputs inputs = driveInputsSupplier.get();
    if (Robot.getInstance().isRedAlliance()) {
      inputs = inputs.redFlipped();
    }
    if (inputs.isNonZero()) { // Simplified: always disable align if driver is giving input
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

    Swerve.getInstance().isAligned = false; // default to false, override later

    ChassisSpeeds speeds;
    if (inputs.getAlignMode() == Controls.AlignMode.None) {
      speeds = chassisSpeedsFromDriveInputs(inputs);
    } else if (inputs.getAlignMode() == Controls.AlignMode.BargeAlign) {
      speeds =
          new ChassisSpeeds(
              fixBargeTranslationInput(
                  xPID.calculate(
                      Swerve.getInstance().getEstimatedPose().getX(),
                      Robot.getInstance().isOnRedSide()
                          ? Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X
                          : Constants.Field.BLUE_BARGE_SCORING_X)),
              0.0,
              fixRotationInput(
                  turnPID.calculate(
                      Swerve.getInstance().getEstimatedPose().getRotation().getRadians(),
                      Math.abs(
                                  Math.PI / 2
                                      - Swerve.getInstance()
                                          .getEstimatedPose()
                                          .getRotation()
                                          .getRadians())
                              < Math.abs(
                                  -Math.PI / 2
                                      - Swerve.getInstance()
                                          .getEstimatedPose()
                                          .getRotation()
                                          .getRadians())
                          ? Math.PI / 2
                          : -Math.PI / 2)));
    } else {
      Pose2d pose = null;
      switch (inputs.getAlignMode()) {
        case TroughAlign:
          pose = Swerve.getInstance().getClosestTroughScoringPose().orElse(null);
          break;
        case ReefAlign:
          pose =
              Swerve.getInstance()
                  .getClosestFudgedScoringPose()
                  .map(Swerve.IndexedPose2d::getPose)
                  .orElse(null);
          break;
        case AlgaeAlign:
          pose = Swerve.getInstance().getClosestAlgaeGrabPose().orElse(null);
          break;
        default:
          break;
      }

      if (pose == null) {
        speeds = chassisSpeedsFromDriveInputs(inputs);
      } else {
        Swerve.getInstance().isAligned =
            Swerve.getInstance().getWithinTolerance(pose.getTranslation());
        speeds =
            new ChassisSpeeds(
                fixTranslationInput(
                    xPID.calculate(Swerve.getInstance().getEstimatedPose().getX(), pose.getX())),
                fixTranslationInput(
                    yPID.calculate(Swerve.getInstance().getEstimatedPose().getY(), pose.getY())),
                fixRotationInput(
                    turnPID.calculate(
                        Swerve.getInstance().getEstimatedPose().getRotation().getRadians(),
                        pose.getRotation().getRadians())));
      }
    }

    Swerve.getInstance().driveFieldRelative(speeds);
  }

  private double fixRotationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.Swerve.maxAlignRotationSpeed,
            Math.min(Constants.Swerve.maxAlignRotationSpeed, n)),
        0.03);
  }

  private double fixTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.Swerve.maxAlignTranslationSpeed,
            Math.min(Constants.Swerve.maxAlignTranslationSpeed, n)),
        0.03);
  }

  private double fixBargeTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        Math.max(
            -Constants.Swerve.maxBargeAlignTranslationSpeed,
            Math.min(Constants.Swerve.maxBargeAlignTranslationSpeed, n)),
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

    double actualX = r * Math.cos(theta) * Constants.Swerve.MAX_VELOCITY;
    double actualY = r * Math.sin(theta) * Constants.Swerve.MAX_VELOCITY;
    double actualRotation = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY;

    return new ChassisSpeeds(actualX, actualY, actualRotation);
  }
}
