package org.chillout1778.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;
import org.chillout1778.Utils;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

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

    private Vision() {
        cameras = new Camera[] {
                new Camera(Constants.Vision.FRONT_RIGHT_NAME, Constants.Vision.FRONT_RIGHT_TRANSFORM),
                new Camera(Constants.Vision.FRONT_LEFT_NAME, Constants.Vision.FRONT_LEFT_TRANSFORM),
                new Camera(Constants.Vision.BACK_RIGHT_NAME, Constants.Vision.BACK_RIGHT_TRANSFORM),
                new Camera(Constants.Vision.BACK_LEFT_NAME, Constants.Vision.BACK_LEFT_TRANSFORM)
        };
    }

    public boolean allConnected() {
        for (Camera camera : cameras) {
            if (!camera.isConnected()) {
                return false;
            }
        }
        return true;
    }

    public boolean removeResult(PhotonPipelineResult res) {
        List<Integer> targetIds = Arrays.asList(4, 5, 14, 15);
        return res.getTargets().stream()
                .map(target -> target.getFiducialId())
                .anyMatch(targetIds::contains);
    }

    public void periodicAddMeasurements(SwerveDrivePoseEstimator estimator) {
        for (Camera camera : cameras) {
            if (camera.isConnected()) {
                List<PhotonPipelineResult> results = camera.getAllUnreadResults().stream()
                        .filter(result -> !removeResult(result))
                        .collect(Collectors.toList());

                for (PhotonPipelineResult result : results) {
                    Optional<PhotonPoseEstimator.PoseEstimate> poseEstimate = camera.getPoseEstimator().update(result);
                    if (poseEstimate.isPresent()) {
                        PhotonPoseEstimator.PoseEstimate pose = poseEstimate.get();
                        if (Utils.isInsideField(pose.getEstimatedPose().getTranslation().toTranslation2d()) &&
                                (pose.getStrategy() == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                                        (pose.getStrategy() == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY &&
                                                pose.getTargetsUsed().get(0).getPoseAmbiguity() < 0.05 &&
                                                pose.getTargetsUsed().get(0).getArea() > 0.25))) {
                            estimator.addVisionMeasurement(
                                    pose.getEstimatedPose().toPose2d(),
                                    pose.getTimestampSeconds()
                            );
                        }
                    }
                }
            }
        }
    }

    @Override
    public void periodic() {
        // Vision measurements are handled in periodicAddMeasurements
        // This method can be used for logging or other periodic tasks
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        for (Camera camera : cameras) {
            builder.addBooleanProperty(
                    camera.getName() + " connection status",
                    camera::isConnected,
                    null);
        }
    }
}
