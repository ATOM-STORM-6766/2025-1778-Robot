package org.chillout1778.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chillout1778.Constants;
import org.chillout1778.Robot;

import java.util.List;

public class Swerve extends SubsystemBase {
    private static Swerve instance;
    
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final Field2d fieldEstimate;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    public boolean isAligned = false;

    private Swerve() {
        gyro = new Pigeon2(Constants.CanIds.GYRO);
        gyro.getConfigurator().apply(new Pigeon2Configuration());

        // The order doesn't matter to us, but PathPlannerLib prefers FL, FR, BL, BR.
        modules = new SwerveModule[] {
            new SwerveModule(
                "Front Left",
                Constants.CanIds.SWERVE_FRONT_LEFT_DRIVE,
                Constants.CanIds.SWERVE_FRONT_LEFT_TURN,
                Constants.CanIds.SWERVE_FRONT_LEFT_CANCODER,
                Constants.Swerve.FRONT_LEFT_ENCODER_OFFSET,
                InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive
            ),
            new SwerveModule(
                "Front Right",
                Constants.CanIds.SWERVE_FRONT_RIGHT_DRIVE,
                Constants.CanIds.SWERVE_FRONT_RIGHT_TURN,
                Constants.CanIds.SWERVE_FRONT_RIGHT_CANCODER,
                Constants.Swerve.FRONT_RIGHT_ENCODER_OFFSET,
                InvertedValue.Clockwise_Positive,
                InvertedValue.Clockwise_Positive
            ),
            new SwerveModule(
                "Back Left",
                Constants.CanIds.SWERVE_BACK_LEFT_DRIVE,
                Constants.CanIds.SWERVE_BACK_LEFT_TURN,
                Constants.CanIds.SWERVE_BACK_LEFT_CANCODER,
                Constants.Swerve.BACK_LEFT_ENCODER_OFFSET,
                InvertedValue.CounterClockwise_Positive,
                InvertedValue.Clockwise_Positive
            ),
            new SwerveModule(
                "Back Right",
                Constants.CanIds.SWERVE_BACK_RIGHT_DRIVE,
                Constants.CanIds.SWERVE_BACK_RIGHT_TURN,
                Constants.CanIds.SWERVE_BACK_RIGHT_CANCODER,
                Constants.Swerve.BACK_RIGHT_ENCODER_OFFSET,
                InvertedValue.Clockwise_Positive,
                InvertedValue.Clockwise_Positive
            )
        };

        fieldEstimate = new Field2d();

        for (SwerveModule module : modules) {
            Shuffleboard.getTab("Swerve").add(module.getName(), module);
        }
        Shuffleboard.getTab("Swerve").add("swerve actual object", this);
        Shuffleboard.getTab("Swerve").add("Swerve Estimated Pose Field", fieldEstimate);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // FL
            new Translation2d(Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // FR
            new Translation2d(-Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // BL
            new Translation2d(-Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE) // BR
        );

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(getGyroAngle()),
            getModulePositions(),
            new Pose2d(0.0, 0.0, new Rotation2d(getGyroAngle())),
            VecBuilder.fill(0.1, 0.1, 0.1), // odometry
            VecBuilder.fill(0.9, 0.9, 2.0) // vision
        );
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(gyro.getRotation2d().getRadians());
    }

    public void setGyroAngle(double desiredAngle) {
        gyro.setYaw(MathUtil.angleModulus(desiredAngle) / Math.PI * 180);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean withinTolerance(Translation2d t) {
        return getEstimatedPose().getTranslation().getDistance(t) < Constants.Swerve.ALIGNMENT_TOLERANCE;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                getEstimatedPose().getRotation()
            )
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.kDefaultPeriod);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].driveState(moduleStates[i]);
        }
        poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
    }    public void stop() {
        driveRobotRelative(new ChassisSpeeds());
    }
    
    // Trajectory following methods
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(new Rotation2d(getGyroAngle()), getModulePositions(), pose);
    }
      public void followSample(Object sample) {
        // Choreo sample following implementation
        // Note: Implementation depends on specific Choreo API version
        // The sample would typically contain velocity and position targets
        // that are used to drive the swerve modules
        
        // Placeholder implementation - in practice this would:
        // 1. Extract velocities from the sample
        // 2. Convert to chassis speeds
        // 3. Drive the robot using those speeds
        
        if (sample != null) {
            // driveFieldRelative(chassisSpeedsFromSample(sample));
            System.out.println("Following trajectory sample");
        }
    }// Alignment and scoring pose methods
    private double scoreForPose(Pose2d pose) {
        // Simple scoring based on distance from current pose
        Pose2d currentPose = getEstimatedPose();
        return currentPose.getTranslation().getDistance(pose.getTranslation());
    }
    
    public Pose2d getClosestTroughScoringPose() {
        List<Pose2d> poses = org.chillout1778.Robot.isRedAlliance() ? 
            Constants.Field.redTroughScoringPoses : Constants.Field.blueTroughScoringPoses;
        
        // Filter poses within reasonable distance
        Pose2d currentPose = getEstimatedPose();
        return poses.stream()
            .filter(pose -> pose.getTranslation().getDistance(currentPose.getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .min((p1, p2) -> Double.compare(scoreForPose(p1), scoreForPose(p2)))
            .orElse(new Pose2d()); // Default fallback
    }

    public class FudgedPose {
        public Pose2d value;
        public FudgedPose(Pose2d pose) { this.value = pose; }
    }    public FudgedPose getClosestFudgedScoringPose() {
        List<Pose2d> poses = org.chillout1778.Robot.isRedAlliance() ? 
            Constants.Field.redScoringPoses : Constants.Field.blueScoringPoses;
        
        Pose2d currentPose = getEstimatedPose();
        Pose2d closestPose = poses.stream()
            .filter(pose -> pose.getTranslation().getDistance(currentPose.getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .min((p1, p2) -> Double.compare(scoreForPose(p1), scoreForPose(p2)))
            .orElse(new Pose2d());
            
        return new FudgedPose(closestPose);
    }

    public Pose2d getClosestAlgaeGrabPose() {
        List<Pose2d> poses = org.chillout1778.Robot.isRedAlliance() ? 
            Constants.Field.redAlgaeGrabbingPose : Constants.Field.blueAlgaeGrabbingPose;
        
        Pose2d currentPose = getEstimatedPose();
        return poses.stream()
            .filter(pose -> pose.getTranslation().getDistance(currentPose.getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .min((p1, p2) -> Double.compare(scoreForPose(p1), scoreForPose(p2)))
            .orElse(new Pose2d()); // Default fallback
    }@Override
    public void periodic() {
        Vision.getInstance().periodicAddMeasurements(poseEstimator);
        poseEstimator.update(
            new Rotation2d(getGyroAngle()),
            getModulePositions()
        );
        fieldEstimate.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve");
        builder.addDoubleProperty("Gyro Angle", this::getGyroAngle, null);
        builder.addBooleanProperty("Is Aligned", () -> isAligned, null);
    }
}
