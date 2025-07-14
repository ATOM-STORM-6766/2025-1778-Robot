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

  // Angle holding variables
  private Rotation2d lastTargetAngle = null;
  private boolean wasRotating = false;

  private Pose2d lastTargetPose = null;

  // 场地中心控制请求：用于自由控制机器人的移动和旋转
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.SwerveDriveKinematics.MAX_VELOCITY * 0.1) // 设置移动死区
      .withRotationalDeadband(Constants.SwerveDriveKinematics.MAX_ANGULAR_VELOCITY * 0.1); // 设置旋转死区

  // 场地中心朝向控制请求：用于保持机器人朝向
  private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(Constants.SwerveDriveKinematics.MAX_VELOCITY * 0.1) // 设置移动死区
      .withRotationalDeadband(Constants.SwerveDriveKinematics.MAX_ANGULAR_VELOCITY * 0.1); // 设置旋转死区

  public TeleopDriveNextCommand(Supplier<Controls.DriveInputs> driveInputsSupplier) {
    this.driveInputsSupplier = driveInputsSupplier;
    addRequirements(swerve);
    
    // 配置角度保持PID参数 - 使用与Constants.SwerveNext.HeadingController相同的参数
    facingAngle.HeadingController.setPID(3.0, 0.0, 0.01);
    facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    Controls.DriveInputs inputs = driveInputsSupplier.get();
    if (Robot.getInstance().isRedAlliance())
      inputs = inputs.redFlipped();

    // 检测旋转输入 - Controls.java已经处理了死区，所以直接检查是否接近零
    double rotationInput = inputs.getRotation();
    boolean isRotating = Math.abs(rotationInput) > inputs.getDeadzone();

    if (inputs.isNonZero()) {
      inputs =
          new Controls.DriveInputs(
              inputs.getForward(),
              inputs.getLeft(),
              inputs.getRotation(),
              inputs.getDeadzone(),
              Controls.AlignMode.None); // if drive inputs non-zero, don't do align
    }
    
    // 如果从旋转状态变为非旋转状态，记录当前角度作为目标角度
    if (wasRotating && !isRotating) {
      lastTargetAngle = swerve.getEstimatedPose().getRotation();
    }
    wasRotating = isRotating;

    if (inputs.getAlignMode() != previousAlignMode) {
      xPID.reset();
      yPID.reset();
      turnPID.reset();
    }
    previousAlignMode = inputs.getAlignMode();

    swerve.setIsAligned(false); // default to false, override later

    // 根据控制模式选择合适的swerve request
    if (inputs.getAlignMode() == Controls.AlignMode.None) {
      // 普通驾驶模式：根据是否有旋转输入选择控制方式
      if (isRotating) {
        // 有旋转输入：使用普通的场地中心控制
        ChassisSpeeds speeds = chassisSpeedsFromDriveInputs(inputs);
        swerve.setControl(
            fieldCentric
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      } else {
        // 无旋转输入：使用角度保持控制
        ChassisSpeeds speeds = chassisSpeedsFromDriveInputs(inputs);
        if (lastTargetAngle != null) {
          swerve.setControl(
              facingAngle
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(lastTargetAngle));
        } else {
          // 如果还没有记录过角度，就使用当前角度
          lastTargetAngle = swerve.getEstimatedPose().getRotation();
          swerve.setControl(
              facingAngle
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(lastTargetAngle));
        }
      }
    } else if (inputs.getAlignMode() == Controls.AlignMode.BargeAlign) {
      // Barge对齐模式
      ChassisSpeeds speeds =
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

        lastTargetPose = new Pose2d(
            Robot.getInstance().isOnRedSide()
                ? Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X
                : Constants.Field.BLUE_BARGE_SCORING_X,
            swerve.getEstimatedPose().getY(),
            new Rotation2d(Math.abs(Math.PI / 2 - swerve.getEstimatedPose().getRotation().getRadians())
                < Math.abs(-Math.PI / 2 - swerve.getEstimatedPose().getRotation().getRadians())
                ? Math.PI / 2
                : -Math.PI / 2));

      swerve.setControl(
          new SwerveRequest.FieldCentric()
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond));
    } else {
      // 其他对齐模式
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
        ChassisSpeeds speeds = chassisSpeedsFromDriveInputs(inputs);
        swerve.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      } else {
        swerve.setIsAligned(swerve.getWithinTolerance(pose.getTranslation()));
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                fixTranslationInput(xPID.calculate(swerve.getEstimatedPose().getX(), pose.getX())),
                fixTranslationInput(yPID.calculate(swerve.getEstimatedPose().getY(), pose.getY())),
                fixRotationInput(
                    turnPID.calculate(
                        swerve.getEstimatedPose().getRotation().getRadians(),
                        pose.getRotation().getRadians())));

        swerve.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
      }
    }
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
    builder.addDoubleArrayProperty("Target Pose", () -> {
      if (lastTargetPose != null) {
        return new double[] {lastTargetPose.getX(), lastTargetPose.getY(), lastTargetPose.getRotation().getRadians()};
      }
      return new double[] {};
    }, null);
    builder.addDoubleProperty("Target Angle", () -> lastTargetAngle != null ? lastTargetAngle.getDegrees() : 0.0, null);


    // Use LogManager to register PID controllers
    LogManager.registerPIDController("Teleop Align X PID", xPID);
    LogManager.registerPIDController("Teleop Align Y PID", yPID);
    LogManager.registerPIDController("Teleop Align Turn PID", turnPID);
  }
}
