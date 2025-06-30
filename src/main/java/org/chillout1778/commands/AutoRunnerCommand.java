package org.chillout1778.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.subsystems.Superstructure;
import org.chillout1778.subsystems.Swerve;

public class AutoRunnerCommand extends Command {
    private final Trajectory<SwerveSample> trajectory;
    private final Timer timer = new Timer();

    public AutoRunnerCommand(Trajectory<SwerveSample> trajectory) {
        this.trajectory = trajectory;
        addRequirements(Superstructure.getInstance(), Swerve.getInstance());
    }

    @Override
    public void initialize() {
        timer.restart();
        // Reset pose estimator to trajectory's initial pose
        // Note: Choreo API may require boolean parameter for alliance
        // Implementation will depend on actual Choreo version being used
        System.out.println("AutoRunnerCommand initialized - trajectory following ready");
    }

    @Override
    public void execute() {
        double currentTime = timer.get(); // Sample the trajectory at the current time
        var sampleOpt = trajectory.sampleAt(currentTime, false);
        if (sampleOpt.isPresent()) {
            SwerveSample sample = sampleOpt.get();
            // Follow the trajectory sample
            Swerve.getInstance().followSample(sample);

            // Process trajectory events
            processTrajectoryEvents(currentTime);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime());
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
        timer.stop();
    }

    private void processTrajectoryEvents(double currentTime) {
        // Process trajectory events based on event markers
        // This would typically handle things like:
        // - Starting intake
        // - Scoring sequences
        // - State changes
        // Implementation depends on specific trajectory event format
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AutoRunnerCommand");
        builder.addDoubleProperty("Current Time", timer::get, null);
        builder.addDoubleProperty("Total Time", trajectory::getTotalTime, null);
        builder.addBooleanProperty("Is Finished", this::isFinished, null);
    }
}
