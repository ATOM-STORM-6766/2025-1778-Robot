package org.chillout1778.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.Constants;
import org.chillout1778.Robot;
import org.chillout1778.subsystems.SwerveNext;

public class AutoRunnerTesterCommand extends Command {
  private final Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();

  public AutoRunnerTesterCommand(Trajectory<SwerveSample> trajectory) {
    this.trajectory = trajectory;
    addRequirements(SwerveNext.getInstance());
  }

  @Override
  public void initialize() {
    timer.restart();
    Pose2d initialPose = trajectory.getInitialPose(Robot.getInstance().isRedAlliance()).get();
    if (SwerveNext.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(initialPose.getTranslation())
        > Constants.SwerveDriveKinematics.STARTING_TOLERANCE) {
      SwerveNext.getInstance().setEstimatedPose(initialPose);
    }
  }

  @Override
  public void execute() {
    SwerveSample sample =
        trajectory
            .sampleAt(timer.get(), Robot.getInstance().isRedAlliance())
            .orElse(trajectory.getFinalSample(Robot.getInstance().isRedAlliance()).get());
    SwerveNext.getInstance().followSample(sample);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    SwerveNext.getInstance().stop();
    timer.stop();
  }
}
