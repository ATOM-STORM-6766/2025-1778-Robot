package org.chillout1778.commands;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.chillout1778.Constants;
import org.chillout1778.Robot;
import org.chillout1778.subsystems.Arm;
import org.chillout1778.subsystems.Elevator;
import org.chillout1778.subsystems.Superstructure;
import org.chillout1778.subsystems.Superstructure.ScoringLevel;
import org.chillout1778.subsystems.SwerveNext;

public class AutoRunnerCommand extends Command {
  private final Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();

  private static class Event {
    final String name;
    final double timestamp;
    final Superstructure.SuperstructureInputs inputs;
    final Supplier<Boolean> waitCondition;
    final boolean requireAlignment;

    Event(
        String name,
        double timestamp,
        Superstructure.SuperstructureInputs inputs,
        Supplier<Boolean> waitCondition,
        boolean requireAlignment) {
      this.name = name;
      this.timestamp = timestamp;
      this.inputs = inputs;
      this.waitCondition = waitCondition;
      this.requireAlignment = requireAlignment;
    }

    Event(String name, Superstructure.SuperstructureInputs inputs) {
      this(name, 0.0, inputs, null, false);
    }

    Event(
        String name,
        Superstructure.SuperstructureInputs inputs,
        Supplier<Boolean> waitCondition,
        boolean requireAlignment) {
      this(name, 0.0, inputs, waitCondition, requireAlignment);
    }

    public Event withTimestamp(double timestamp) {
      return new Event(
          this.name, timestamp, this.inputs, this.waitCondition, this.requireAlignment);
    }
  }

  private final List<Event> eventTypes =
      List.of(
          new Event(
              "startL4",
              new Superstructure.SuperstructureInputs(
                  true,
                  false,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event(
              "L4",
              new Superstructure.SuperstructureInputs(
                  true,
                  false,
                  false,
                  false,
                  true,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false),
              () ->
                  !Arm.getInstance().getHasObject()
                      || (Superstructure.getInstance().state == Superstructure.State.PlaceL4
                          && Arm.getInstance().isAtSetpoint()
                          && Elevator.getInstance().isAtSetpoint()),
              true),
          new Event(
              "intakeDown",
              new Superstructure.SuperstructureInputs(
                  false,
                  true,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event(
              "intakeDownNoHandoff",
              new Superstructure.SuperstructureInputs(
                  false,
                  true,
                  false,
                  false,
                  false,
                  ScoringLevel.TROUGH,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event("intakeUp", new Superstructure.SuperstructureInputs()), // Zero inputs
          new Event("zeroInputs", new Superstructure.SuperstructureInputs()), // Zero inputs
          new Event(
              "startGetAlgae",
              new Superstructure.SuperstructureInputs(
                  false,
                  false,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  true,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event(
              "waitGetAlgae",
              new Superstructure.SuperstructureInputs(
                  false,
                  false,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  true,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false),
              () -> Arm.getInstance().getHasObject(),
              true),
          new Event(
              "extendAlgae",
              new Superstructure.SuperstructureInputs(
                  true,
                  false,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event(
              "scoreAlgae",
              new Superstructure.SuperstructureInputs(
                  true,
                  false,
                  false,
                  false,
                  true,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false)),
          new Event(
              "verticalCoral",
              new Superstructure.SuperstructureInputs(
                  false,
                  false,
                  false,
                  false,
                  false,
                  ScoringLevel.L4,
                  false,
                  false,
                  false,
                  false,
                  false,
                  false,
                  true)));

  private Event eventFromEventMarker(EventMarker ev) {
    for (Event type : eventTypes) {
      if (type.requireAlignment) {
        assert (type.waitCondition != null);
      }
      if (type.name.equals(ev.event)) {
        return type.withTimestamp(ev.timestamp);
      }
    }
    throw new RuntimeException("Unrecognized event marker: " + ev.event);
  }

  private List<Event> events;
  private int eventI = 0;
  private Event currentWaitEvent = null;
  private boolean waitingForAlign = false;
  private Superstructure.SuperstructureInputs postAlignInputs = null;
  private Pose2d lastPose = null;

  public AutoRunnerCommand(Trajectory<SwerveSample> trajectory) {
    this.trajectory = trajectory;
    this.events =
        trajectory.events().stream()
            .sorted(Comparator.comparingDouble(e -> e.timestamp))
            .map(this::eventFromEventMarker)
            .collect(Collectors.toList());
    addRequirements(Superstructure.getInstance(), SwerveNext.getInstance());
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

  private boolean shouldRunEvent(Event ev) {
    return timer.hasElapsed(ev.timestamp) || timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void execute() {
    if (waitingForAlign) {
      assert (lastPose != null);
      if (SwerveNext.getInstance().getWithinTolerance(lastPose.getTranslation())) {
        Superstructure.getInstance().setInputs(postAlignInputs);
        waitingForAlign = false;
      }
      SwerveNext.getInstance().followPose(lastPose);
      return;
    }

    if (currentWaitEvent != null && currentWaitEvent.waitCondition.get()) {
      Superstructure.getInstance().emptyInputs();
      currentWaitEvent = null;
    } else if (currentWaitEvent == null
        && eventI < events.size()
        && shouldRunEvent(events.get(eventI))) {
      Event ev = events.get(eventI++);
      if (ev.requireAlignment) {
        postAlignInputs = ev.inputs;
      } else {
        Superstructure.getInstance().setInputs(ev.inputs);
      }
      if (ev.waitCondition != null) {
        currentWaitEvent = ev;
        waitingForAlign = ev.requireAlignment;
        SwerveNext.getInstance().stop();
        timer.stop();
      }
    }

    if (currentWaitEvent == null) {
      timer.start();
      SwerveSample sample =
          trajectory
              .sampleAt(timer.get(), Robot.getInstance().isRedAlliance())
              .orElse(trajectory.getFinalSample(Robot.getInstance().isRedAlliance()).get());
      lastPose = sample.getPose();
      SwerveNext.getInstance().followSample(sample);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime())
        && currentWaitEvent == null
        && eventI >= events.size();
  }

  @Override
  public void end(boolean interrupted) {
    SwerveNext.getInstance().stop();
    Superstructure.getInstance().emptyInputs();
    timer.stop();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("AutoRunnerCommand");
    builder.addBooleanProperty("Timer Running", () -> currentWaitEvent == null, null);
    builder.addDoubleProperty("Timer Time", timer::get, null);
    builder.addDoubleProperty("Trajectory Total Time", trajectory::getTotalTime, null);
    builder.addStringProperty(
        "Current Wait Event",
        () -> (currentWaitEvent != null) ? currentWaitEvent.name : "(none)",
        null);
  }
}
