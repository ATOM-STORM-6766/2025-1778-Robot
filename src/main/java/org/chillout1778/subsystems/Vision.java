package org.chillout1778.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.chillout1778.Constants;
import org.chillout1778.Utils;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  public static class Camera extends PhotonCamera {
    private final PhotonPoseEstimator poseEstimator;

    public Camera(String initialName, Transform3d robotToCamera) {
      super(initialName);

      this.poseEstimator = new PhotonPoseEstimator(
          AprilTagFieldLayout.loadField(Constants.Vision.FIELD_TYPE),
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamera);
      poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
      poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPoseEstimator getPoseEstimator() {
      return poseEstimator;
    }
  }

  private final Camera[] cameras;
  private final SwerveNext swerve;

  // 记录每个相机的最新估计位置
  private Pose2d latestEstimatedPose;

  private Vision() {
    swerve = SwerveNext.getInstance();

    cameras = new Camera[] {
        new Camera(Constants.Vision.FRONT_RIGHT_NAME, Constants.Vision.FRONT_RIGHT_TRANSFORM),
        new Camera(Constants.Vision.FRONT_LEFT_NAME, Constants.Vision.FRONT_LEFT_TRANSFORM),
        new Camera(Constants.Vision.BACK_RIGHT_NAME, Constants.Vision.BACK_RIGHT_TRANSFORM),
        new Camera(Constants.Vision.BACK_LEFT_NAME, Constants.Vision.BACK_LEFT_TRANSFORM)
    };

    // 初始化记录数组
    latestEstimatedPose = new Pose2d();

  }

  @Override
  public void periodic() {
    updateVision();
  }

  private void updateVision() {
    for (int i = 0; i < cameras.length; i++) {
      Camera camera = cameras[i];
      if (camera.isConnected()) {
        camera.getAllUnreadResults().stream()
            .filter(result -> !removeResult(result))
            .map(result -> camera.getPoseEstimator().update(result))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .filter(
                pose -> Utils.isInsideField(pose.estimatedPose.getTranslation().toTranslation2d())
                    && (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                        || (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                            && pose.targetsUsed.get(0).getPoseAmbiguity() < 0.05
                            && pose.targetsUsed.get(0).getArea() > 0.25)))
            .forEach(
                pose -> {
                  // 记录最新的估计位置
                  latestEstimatedPose = pose.estimatedPose.toPose2d();

                  // 向 swerve 添加视觉测量
                  swerve.addVisionMeasurement(
                      pose.estimatedPose.toPose2d(), com.ctre.phoenix6.Utils.fpgaToCurrentTime(pose.timestampSeconds));
                });
      }
    }
  }

  public boolean removeResult(PhotonPipelineResult res) {
    List<Integer> targetIds = Arrays.asList(4, 5, 14, 15);
    return res.getTargets().stream()
        .map(target -> target.getFiducialId())
        .anyMatch(targetIds::contains);
  }

  public boolean allConnected() {
    for (Camera camera : cameras) {
      if (!camera.isConnected()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // 相机连接状态
    builder.addBooleanProperty("All Cameras Connected", this::allConnected, null);
    builder.addDoubleArrayProperty("Latest Estimated Poses", () -> {
      double[] poses = new double[3];
      poses[0] = latestEstimatedPose.getX();
      poses[1] = latestEstimatedPose.getY();
      poses[2] = latestEstimatedPose.getRotation().getDegrees();
      return poses;
    }, null);
  }
}
