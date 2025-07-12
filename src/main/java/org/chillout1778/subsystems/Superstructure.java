package org.chillout1778.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.chillout1778.Robot;
import org.chillout1778.commands.ZeroArmCommand;
import org.chillout1778.commands.ZeroElevatorCommand;
import org.chillout1778.commands.ZeroIntakeCommand;

public class Superstructure extends SubsystemBase {
  public enum State {
    StartPosition(
        Elevator.State.Down,
        Arm.PivotState.Up,
        Arm.RollerState.SlowIdle,
        Intake.PivotState.Up,
        Intake.RollerState.Off),
    Rest(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Idle),
    PrePopsiclePickup(
        Elevator.State.PreHandoff,
        Arm.PivotState.PopsiclePickup,
        Arm.RollerState.In,
        Intake.PivotState.Down,
        Intake.RollerState.Off),
    PopsiclePickup(
        Elevator.State.PopsiclePickup,
        Arm.PivotState.PopsiclePickup,
        Arm.RollerState.In,
        Intake.PivotState.Down,
        Intake.RollerState.Off),
    ArmSourceIntake(Elevator.State.Down, Arm.PivotState.Up, Arm.RollerState.In),
    SourceIntake(
        Elevator.State.SourceIntake,
        Arm.PivotState.Down,
        Arm.RollerState.Idle,
        Intake.PivotState.Up,
        Intake.RollerState.In),
    PreHandoff(
        Elevator.State.Handoff,
        Arm.PivotState.Down,
        Arm.RollerState.In,
        Intake.PivotState.Up,
        Intake.RollerState.In),
    Handoff(
        Elevator.State.Handoff,
        Arm.PivotState.Down,
        Arm.RollerState.In,
        Intake.PivotState.Up,
        Intake.RollerState.Out),
    PreScore(
        Elevator.State.PreScore,
        Arm.PivotState.Up,
        Arm.RollerState.Idle,
        Intake.PivotState.Up,
        Intake.RollerState.Off),
    ReverseHandoff(
        Elevator.State.PreHandoff,
        Arm.PivotState.Down,
        Arm.RollerState.Out,
        Intake.PivotState.Up,
        Intake.RollerState.In),
    PreTrough(
        Elevator.State.Trough,
        Arm.PivotState.Down,
        Arm.RollerState.Idle,
        Intake.PivotState.Trough,
        Intake.RollerState.Off),
    Trough(
        Elevator.State.Trough,
        Arm.PivotState.Down,
        Arm.RollerState.Idle,
        Intake.PivotState.Trough,
        Intake.RollerState.TroughOut),
    PrepareL4(Elevator.State.L4, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
    StartL4(Elevator.State.L4, Arm.PivotState.L4ScoreCoral, Arm.RollerState.SlowIdle),
    PlaceL4(Elevator.State.ScoreL4, Arm.PivotState.L4FinishScoreCoral, Arm.RollerState.SlowOut),
    AfterL4(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Out),

    PrepareL3(Elevator.State.L3, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
    StartL3(Elevator.State.L3, Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
    PlaceL3(Elevator.State.ScoreL3, Arm.PivotState.FinishScoreCoral, Arm.RollerState.SlowOut),
    AfterL3(Elevator.State.PostL3, Arm.PivotState.Up, Arm.RollerState.Out),

    PrepareL2(Elevator.State.L2, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
    StartL2(Elevator.State.L2, Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
    PlaceL2(Elevator.State.ScoreL2, Arm.PivotState.FinishScoreCoral, Arm.RollerState.SlowOut),
    AfterL2(Elevator.State.PostL2, Arm.PivotState.Up, Arm.RollerState.Out),

    PreGetAlgae(Elevator.State.HighAlgae, Arm.PivotState.SafeInsideRobotAngle, Arm.RollerState.In),
    GetAlgae(Elevator.State.AutoAlgae, Arm.PivotState.GetAlgae, Arm.RollerState.In),
    PostGetAlgae(Elevator.State.AutoAlgae, Arm.PivotState.PostAlgae, Arm.RollerState.AlgaeIdle),
    AlgaeRest(Elevator.State.AlgaeRest, Arm.PivotState.AlgaeUp, Arm.RollerState.AlgaeIdle),

    PreBarge(Elevator.State.Barge, Arm.PivotState.PreBarge, Arm.RollerState.AlgaeIdle),
    ScoreBarge(Elevator.State.Barge, Arm.PivotState.BargeScore, Arm.RollerState.Out),
    AlgaeDescore(Elevator.State.AutoAlgae, Arm.PivotState.DescoreAlgae, Arm.RollerState.Descore),
    AlgaeExit(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Out),

    PreProcessor(Elevator.State.Processor, Arm.PivotState.Processor, Arm.RollerState.AlgaeIdle),
    ScoreProcessor(Elevator.State.Processor, Arm.PivotState.Processor, Arm.RollerState.SlowOut),
    PreAlgaeGroundIntake(
        Rest.elevator,
        Arm.PivotState.AlgaeGroundPickup,
        Arm.RollerState.Off,
        Intake.PivotState.Down,
        Intake.RollerState.Off),
    AlgaeGroundIntake(
        Elevator.State.GroundAlgaeIntake,
        Arm.PivotState.AlgaeGroundPickup,
        Arm.RollerState.In,
        Intake.PivotState.Down,
        Intake.RollerState.Off),
    ExitAlgaeGroundIntake(
        Elevator.State.PreHandoff,
        Arm.PivotState.ExitAlgaeGroundPickup,
        Arm.RollerState.AlgaeIdle,
        Intake.PivotState.Down,
        Intake.RollerState.Off);

    public final Elevator.State elevator;
    public final Arm.PivotState armPivot;
    public final Arm.RollerState armRollers;
    public final Intake.PivotState intakePivot;
    public final Intake.RollerState intakeRollers;

    State(Elevator.State elevator, Arm.PivotState armPivot, Arm.RollerState armRollers) {
      this(
          elevator,
          armPivot,
          armRollers,
          Intake.PivotState.OperatorControl,
          Intake.RollerState.OperatorControl);
    }

    State(
        Elevator.State elevator,
        Arm.PivotState armPivot,
        Arm.RollerState armRollers,
        Intake.PivotState intakePivot,
        Intake.RollerState intakeRollers) {
      this.elevator = elevator;
      this.armPivot = armPivot;
      this.armRollers = armRollers;
      this.intakePivot = intakePivot;
      this.intakeRollers = intakeRollers;
    }
  }

  public enum ScoringLevel {
    TROUGH,
    L2,
    L3,
    L4;

    public int getIndex() {
      switch (this) {
        case TROUGH:
          return 0;
        case L2:
          return 1;
        case L3:
          return 2;
        case L4:
          return 3;
      }
      return -1;
    }
  }

  public static class SuperstructureInputs {
    public final boolean wantExtend;
    public final boolean wantGroundIntake;
    public final boolean wantArmSourceIntake;
    public final boolean wantSourceIntake;
    public final boolean wantScore;
    public final ScoringLevel wantedScoringLevel;
    public final boolean wantGetAlgae;
    public final boolean wantDescoreAlgae;
    public final boolean wantVerticalPickup;
    public final boolean wantResetSuperstructure;
    public final boolean wantScoreProcessor;
    public final boolean wantAlgaeGroundIntake;
    public final boolean wantPopsiclePickup;

    public boolean getWantExtend() {
      return wantExtend;
    }

    public boolean wantGroundIntake() {
      return wantGroundIntake;
    }

    public boolean getWantArmSourceIntake() {
      return wantArmSourceIntake;
    }

    public boolean getWantSourceIntake() {
      return wantSourceIntake;
    }

    public boolean getWantScore() {
      return wantScore;
    }

    public ScoringLevel getWantedScoringLevel() {
      return wantedScoringLevel;
    }

    public boolean getWantGetAlgae() {
      return wantGetAlgae;
    }

    public boolean getWantDescoreAlgae() {
      return wantDescoreAlgae;
    }

    public boolean getWantVerticalPickup() {
      return wantVerticalPickup;
    }

    public boolean getWantResetSuperstructure() {
      return wantResetSuperstructure;
    }

    public boolean getWantScoreProcessor() {
      return wantScoreProcessor;
    }

    public boolean getWantAlgaeGroundIntake() {
      return wantAlgaeGroundIntake;
    }

    public boolean getWantPopsiclePickup() {
      return wantPopsiclePickup;
    }

    public SuperstructureInputs() {
      this(
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
          false);
    }

    public SuperstructureInputs(
        boolean wantExtend,
        boolean wantGroundIntake,
        boolean wantArmSourceIntake,
        boolean wantSourceIntake,
        boolean wantScore,
        ScoringLevel wantedScoringLevel,
        boolean wantGetAlgae,
        boolean wantDescoreAlgae,
        boolean wantVerticalPickup,
        boolean wantResetSuperstructure,
        boolean wantScoreProcessor,
        boolean wantAlgaeGroundIntake,
        boolean wantPopsiclePickup) {
      this.wantExtend = wantExtend;
      this.wantGroundIntake = wantGroundIntake;
      this.wantArmSourceIntake = wantArmSourceIntake;
      this.wantSourceIntake = wantSourceIntake;
      this.wantScore = wantScore;
      this.wantedScoringLevel = wantedScoringLevel;
      this.wantGetAlgae = wantGetAlgae;
      this.wantDescoreAlgae = wantDescoreAlgae;
      this.wantVerticalPickup = wantVerticalPickup;
      this.wantResetSuperstructure = wantResetSuperstructure;
      this.wantScoreProcessor = wantScoreProcessor;
      this.wantAlgaeGroundIntake = wantAlgaeGroundIntake;
      this.wantPopsiclePickup = wantPopsiclePickup;
    }
  }

  public static class Transition {
    public final State cur;
    public final State next;
    public final Runnable enterFunction;
    public final BooleanSupplier transitionCheck;

    public Transition(State cur, State next, BooleanSupplier transitionCheck) {
      this(cur, next, () -> {}, transitionCheck);
    }

    public Transition(
        State cur, State next, Runnable enterFunction, BooleanSupplier transitionCheck) {
      this.cur = cur;
      this.next = next;
      this.enterFunction = enterFunction;
      this.transitionCheck = transitionCheck;
    }
  }

  public static final double popsicleDelay = 0.5;

  private static Superstructure instance;

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  public SuperstructureInputs inputs = new SuperstructureInputs();
  public Timer stateTimer = new Timer();
  public State state = State.StartPosition;

  public ScoringLevel getWantedScoringLevel() {
    return inputs.wantedScoringLevel;
  }

  public void setInputs(SuperstructureInputs inputs) {
    this.inputs = inputs;
  }

  public SuperstructureInputs getInputs() {
    return inputs;
  }

  private final List<Transition> transitions = new ArrayList<>();

  private Superstructure() {
    // transitions 初始化
    transitions.add(
        new Transition(
            State.StartPosition,
            State.Rest,
            () -> inputs.wantGroundIntake || inputs.wantArmSourceIntake));
    transitions.add(
        new Transition(
            State.StartPosition, State.PreScore, () -> Robot.getInstance().isAutonomous()));
    transitions.add(
        new Transition(State.Rest, State.ArmSourceIntake, () -> inputs.wantArmSourceIntake));
    transitions.add(
        new Transition(
            State.ArmSourceIntake,
            State.Rest,
            () -> !inputs.wantArmSourceIntake || Arm.getInstance().getHasObject()));
    transitions.add(new Transition(State.Rest, State.SourceIntake, () -> inputs.wantSourceIntake));
    transitions.add(
        new Transition(
            State.SourceIntake,
            State.Rest,
            () -> !inputs.wantSourceIntake || Intake.getInstance().hasCoral()));
    transitions.add(
        new Transition(
            State.PreScore,
            State.Rest,
            () ->
                inputs.wantedScoringLevel == ScoringLevel.TROUGH
                    || !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.Rest,
            State.ReverseHandoff,
            () ->
                Arm.getInstance().isAtSetpoint()
                    && Elevator.getInstance().isAtSetpoint()
                    && inputs.wantedScoringLevel == ScoringLevel.TROUGH
                    && Arm.getInstance().getHasObject()
                    && !Intake.getInstance().hasCoral()
                    && Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Up
                    && Intake.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(
            State.ReverseHandoff,
            State.Rest,
            () -> Intake.getInstance().hasCoral() || inputs.wantResetSuperstructure));
    transitions.add(
        new Transition(
            State.Rest,
            State.PreTrough,
            () ->
                inputs.wantExtend
                    && inputs.wantedScoringLevel == ScoringLevel.TROUGH
                    && Elevator.getInstance().isAtOrAboveSetpoint()
                    && Arm.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(
            State.PreTrough,
            State.Trough,
            () -> Intake.getInstance().isAtSetpoint() && inputs.wantScore));
    transitions.add(new Transition(State.PreTrough, State.Rest, () -> !inputs.wantExtend));
    transitions.add(new Transition(State.Trough, State.Rest, () -> !inputs.wantScore));
    transitions.add(
        new Transition(
            State.Rest,
            State.PreHandoff,
            () ->
                Elevator.getInstance().isAtSetpoint()
                    && Arm.getInstance().isAtSetpoint()
                    && inputs.wantedScoringLevel != ScoringLevel.TROUGH
                    && Intake.getInstance().hasCoral()));
    transitions.add(
        new Transition(
            State.PreHandoff,
            State.Handoff,
            () ->
                Elevator.getInstance().isAtSetpoint()
                    && Arm.getInstance().isAtSetpoint()
                    && Intake.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(State.Handoff, State.Rest, () -> Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.Rest,
            State.PreScore,
            () ->
                Arm.getInstance().getHasObject()
                    && inputs.wantedScoringLevel != ScoringLevel.TROUGH));
    transitions.add(
        new Transition(State.Handoff, State.Rest, () -> inputs.wantResetSuperstructure));
    transitions.add(
        new Transition(
            State.PreScore,
            State.PrepareL4,
            () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L4));
    transitions.add(
        new Transition(
            State.PreScore,
            State.PrepareL3,
            () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L3));
    transitions.add(
        new Transition(
            State.PreScore,
            State.PrepareL2,
            () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L2));
    addScoringTransitions(
        ScoringLevel.L4, State.PrepareL4, State.StartL4, State.PlaceL4, State.AfterL4);
    addScoringTransitions(
        ScoringLevel.L3, State.PrepareL3, State.StartL3, State.PlaceL3, State.AfterL3);
    addScoringTransitions(
        ScoringLevel.L2, State.PrepareL2, State.StartL2, State.PlaceL2, State.AfterL2);
    transitions.add(
        new Transition(
            State.AlgaeExit,
            State.PreGetAlgae,
            () -> inputs.wantGetAlgae && !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.Rest,
            State.PreGetAlgae,
            () -> inputs.wantGetAlgae && !Arm.getInstance().getHasObject()));
    transitions.add(new Transition(State.PreGetAlgae, State.Rest, () -> !inputs.wantGetAlgae));
    transitions.add(
        new Transition(
            State.PreGetAlgae, State.GetAlgae, () -> Elevator.getInstance().isAtSetpoint()));
    transitions.add(new Transition(State.GetAlgae, State.PreGetAlgae, () -> !inputs.wantGetAlgae));
    transitions.add(
        new Transition(State.GetAlgae, State.PostGetAlgae, () -> Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.PostGetAlgae,
            State.AlgaeRest,
            () -> Arm.getInstance().isAtSetpoint() && Arm.getInstance().atSafeReefDistance()));
    transitions.add(
        new Transition(State.AlgaeRest, State.AlgaeExit, () -> !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.AlgaeExit,
            State.Rest,
            () -> Arm.getInstance().isAtSetpoint() && Elevator.getInstance().isAtSetpoint()));
    transitions.add(new Transition(State.AlgaeRest, State.PreBarge, () -> inputs.wantExtend));
    transitions.add(new Transition(State.PreBarge, State.AlgaeRest, () -> !inputs.wantExtend));
    transitions.add(
        new Transition(
            State.PreBarge,
            State.ScoreBarge,
            () -> inputs.wantScore && SwerveNext.getInstance().atGoodScoringDistance()));
    transitions.add(
        new Transition(
            State.ScoreBarge,
            State.PreBarge,
            () ->
                (!inputs.wantExtend || !Arm.getInstance().getHasObject())
                    && Arm.getInstance().atSafeBargeDistance()));
    transitions.add(new Transition(State.Rest, State.AlgaeDescore, () -> inputs.wantDescoreAlgae));
    transitions.add(new Transition(State.AlgaeDescore, State.Rest, () -> !inputs.wantDescoreAlgae));
    transitions.add(
        new Transition(State.AlgaeRest, State.PreProcessor, () -> inputs.wantScoreProcessor));
    transitions.add(
        new Transition(State.PreProcessor, State.ScoreProcessor, () -> inputs.wantScore));
    transitions.add(
        new Transition(
            State.ScoreProcessor,
            State.AlgaeRest,
            () ->
                !inputs.wantScoreProcessor
                    && !Arm.getInstance().getHasObject()
                    && Arm.getInstance().atSafeProcessorDistance()));
    transitions.add(
        new Transition(
            State.PreProcessor,
            State.AlgaeRest,
            () -> !inputs.wantScoreProcessor && Arm.getInstance().atSafeProcessorDistance()));
    transitions.add(
        new Transition(
            State.Rest,
            State.PreAlgaeGroundIntake,
            () -> inputs.wantAlgaeGroundIntake && !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.PreAlgaeGroundIntake,
            State.AlgaeGroundIntake,
            () -> inputs.wantAlgaeGroundIntake && Intake.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(
            State.AlgaeGroundIntake,
            State.ExitAlgaeGroundIntake,
            () -> !inputs.wantAlgaeGroundIntake || Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.ExitAlgaeGroundIntake,
            State.AlgaeRest,
            () -> Elevator.getInstance().isAtSetpoint() && Arm.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(
            State.PopsiclePickup,
            State.PrePopsiclePickup,
            () ->
                !inputs.wantPopsiclePickup
                    || (Robot.getInstance().isAutonomous()
                        && stateTimer.hasElapsed(popsicleDelay))));
    transitions.add(
        new Transition(State.PrePopsiclePickup, State.Rest, () -> !inputs.wantPopsiclePickup));
    transitions.add(
        new Transition(
            State.Rest,
            State.PrePopsiclePickup,
            () -> inputs.wantPopsiclePickup && !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            State.PrePopsiclePickup,
            State.PopsiclePickup,
            () -> inputs.wantPopsiclePickup && Intake.getInstance().isAtSetpoint()));
    transitions.add(
        new Transition(
            State.PopsiclePickup,
            State.PreScore,
            () -> stateTimer.hasElapsed(popsicleDelay) && Arm.getInstance().getHasObject()));
  }

  private void addScoringTransitions(
      ScoringLevel scoringLevel, State prepare, State start, State place, State after) {
    transitions.add(
        new Transition(
            prepare,
            State.PreScore,
            () ->
                Arm.getInstance().isAtSetpoint()
                    && (!Arm.getInstance().getHasObject()
                        || !inputs.wantExtend
                        || inputs.wantedScoringLevel != scoringLevel)));
    transitions.add(
        new Transition(
            start,
            prepare,
            () ->
                !inputs.wantExtend
                    || inputs.wantedScoringLevel != scoringLevel
                    || !Arm.getInstance().getHasObject()));
    transitions.add(
        new Transition(
            prepare,
            start,
            () ->
                Elevator.getInstance().isLazierAtSetpoint()
                    && Arm.getInstance().getHasObject()
                    && inputs.wantExtend
                    && inputs.wantedScoringLevel == scoringLevel));
    transitions.add(
        new Transition(
            start,
            place,
            () -> {
              SwerveNext.getInstance().markPoseScored();
              return Elevator.getInstance().isAtSetpoint()
                  && Arm.getInstance().isAtSetpoint()
                  && inputs.wantScore;
            }));
    transitions.add(
        new Transition(
            place,
            after,
            () ->
                Elevator.getInstance().isAtSetpoint()
                    && Arm.getInstance().isAtSetpoint()
                    && ((place == State.PlaceL2 || place == State.PlaceL3)
                        ? Arm.getInstance().atSafePlacementDistance()
                        : true)));
    transitions.add(new Transition(after, State.Rest, () -> Arm.getInstance().isInsideFrame()));
  }

  public void emptyInputs() {
    inputs = new SuperstructureInputs();
  }

  public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
    return new ParallelCommandGroup(
        new ZeroIntakeCommand(), new ZeroElevatorCommand(), new ZeroArmCommand());
  }

  public void setStates() {
    Arm.getInstance().setState(state.armPivot, state.armRollers);
    Elevator.getInstance().setState(state.elevator);
    Intake.getInstance().setState(state.intakePivot, state.intakeRollers);
  }

  @Override
  public void periodic() {
    if (!Elevator.getInstance().getIsZeroed()
        || !Intake.getInstance().isZeroed()
        || !Arm.getInstance().getIsZeroed()) {
      return;
    }
    stateTimer.start();
    for (Transition transition : transitions) {
      if (transition.cur == state && transition.transitionCheck.getAsBoolean()) {
        state = transition.next;
        transition.enterFunction.run();
        setStates();
        return;
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.toString(), null);
    builder.addBooleanProperty("wantExtend", () -> inputs.wantExtend, null);
    builder.addBooleanProperty("wantScore", () -> inputs.wantScore, null);
    builder.addStringProperty("scoringLevel", () -> inputs.wantedScoringLevel.toString(), null);
  }
}
