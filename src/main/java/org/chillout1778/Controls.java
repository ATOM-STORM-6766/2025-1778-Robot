package org.chillout1778;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.chillout1778.subsystems.Arm;
import org.chillout1778.subsystems.Elevator;
import org.chillout1778.subsystems.Superstructure;

public class Controls {
  private static final CommandGenericHID driverController = new CommandGenericHID(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  public enum AlignMode {
    None,
    ReefAlign,
    TroughAlign,
    AlgaeAlign,
    BargeAlign
  }

  public static class DriveInputs {
    private final double forward;
    private final double left;
    private final double rotation;
    private final double deadzone;
    private final AlignMode alignMode;

    public DriveInputs(
        double forward, double left, double rotation, double deadzone, AlignMode alignMode) {
      this.forward = forward;
      this.left = left;
      this.rotation = rotation;
      this.deadzone = deadzone;
      this.alignMode = alignMode;
    }

    public double getForward() {
      return forward;
    }

    public double getLeft() {
      return left;
    }

    public double getRotation() {
      return rotation;
    }

    public double getDeadzone() {
      return deadzone;
    }

    public AlignMode getAlignMode() {
      return alignMode;
    }

    public boolean isNonZero() {
      return Math.abs(forward) > deadzone
          || Math.abs(left) > deadzone
          || Math.abs(rotation) > deadzone;
    }

    public DriveInputs redFlipped() {
      return new DriveInputs(-forward, -left, rotation, deadzone, alignMode);
    }
  }

  public static DriveInputs emptyInputs() {
    return new DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None);
  }

  public static DriveInputs driverInputs() {
    return new DriveInputs(
        -driverController.getRawAxis(1),
        -driverController.getRawAxis(0),
        -driverController.getRawAxis(4),
        0.05,
        getDriverAlignMode());
  }

  public static DriveInputs operatorInputs() {
    return new DriveInputs(
        -operatorController.getHID().getLeftY(),
        -operatorController.getHID().getLeftX(),
        -operatorController.getHID().getRightX(),
        0.1,
        getOperatorAlignMode());
  }

  private static AlignMode getDriverAlignMode() {
    if (wantBargeAutoAlign()) return AlignMode.BargeAlign;
    else if (wantCoralAutoAlign()
        && superstructureInputs().getWantedScoringLevel() != Superstructure.ScoringLevel.TROUGH)
      return AlignMode.ReefAlign;
    else if (wantCoralAutoAlign()
        && superstructureInputs().getWantedScoringLevel() == Superstructure.ScoringLevel.TROUGH)
      return AlignMode.TroughAlign;
    else if (wantAlgaeAutoAlign()
        && superstructureInputs().getWantGetAlgae()
        && !Arm.getInstance().getHasObject()) return AlignMode.AlgaeAlign;
    else return AlignMode.None;
  }

  private static AlignMode getOperatorAlignMode() {
    if (wantBargeAutoAlign()) return AlignMode.BargeAlign;
    else if (wantCoralAutoAlign()
        && superstructureInputs().getWantedScoringLevel() != Superstructure.ScoringLevel.TROUGH)
      return AlignMode.ReefAlign;
    else if (wantCoralAutoAlign()
        && superstructureInputs().getWantedScoringLevel() == Superstructure.ScoringLevel.TROUGH)
      return AlignMode.TroughAlign;
    else if (wantAlgaeAutoAlign()
        && superstructureInputs().getWantGetAlgae()
        && !Arm.getInstance().getHasObject()) return AlignMode.AlgaeAlign;
    else return AlignMode.None;
  }

  public static boolean wantCoralAutoAlign() {
    return superstructureInputs().getWantExtend();
  }

  public static boolean wantAlgaeAutoAlign() {
    return superstructureInputs().getWantGetAlgae()
        && Arm.getInstance().isAtSetpoint()
        && Elevator.getInstance().isAtSetpoint();
  }

  public static boolean wantBargeAutoAlign() {
    return superstructureInputs().getWantExtend()
        && (Superstructure.getInstance().state == Superstructure.State.AlgaeRest
            || Superstructure.getInstance().state == Superstructure.State.PreBarge
            || Superstructure.getInstance().state == Superstructure.State.ScoreBarge);
  }

  public static boolean wantOffsetArmPositive() {
    return operatorController.getHID().getLeftX() > 0.9
        && operatorController.getHID().getL3Button();
  }

  public static boolean wantOffsetArmNegative() {
    return operatorController.getHID().getLeftX() < -0.9
        && operatorController.getHID().getL3Button();
  }

  private static Superstructure.ScoringLevel lastScoringLevel = Superstructure.ScoringLevel.TROUGH;

  public static Superstructure.SuperstructureInputs superstructureInputs() {
    Superstructure.ScoringLevel level;
    switch (operatorController.getHID().getPOV()) {
      case 0:
        level = Superstructure.ScoringLevel.L4;
        break;
      case 270:
        level = Superstructure.ScoringLevel.L3;
        break;
      case 180:
        level = Superstructure.ScoringLevel.L2;
        break;
      case 90:
        level = Superstructure.ScoringLevel.TROUGH;
        break;
      default:
        level = lastScoringLevel;
        break;
    }
    lastScoringLevel = level;
    return new Superstructure.SuperstructureInputs(
        operatorController.getHID().getL2Button(), // wantExtend
        operatorController.getHID().getR2Button(), // wantGroundIntake
        operatorController.getHID().getCrossButton(), // wantArmSourceIntake
        operatorController.getHID().getSquareButton(), // wantSourceIntake
        (driverController.getRawAxis(4) > .5
            || operatorController.getHID().getR3Button()), // wantScore
        level, // wantedScoringLevel
        operatorController.getHID().getR1Button(), // wantGetAlgae
        operatorController.getHID().getTriangleButton(), // wantDescoreAlgae
        false, // wantVerticalPickup
        operatorController.getHID().getOptionsButton(), // wantResetSuperstructure
        operatorController.getHID().getCircleButton(), // wantScoreProcessor
        operatorController.getHID().getL1Button(), // wantAlgaeGroundIntake
        false // wantPopsiclePickup
        );
  }
}
