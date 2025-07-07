package org.chillout1778.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.Constants;
import org.chillout1778.subsystems.Intake;

public class ZeroIntakeCommand extends Command {
  private final boolean forced;
  private boolean shouldZero = false;

  public ZeroIntakeCommand() {
    this(false);
  }

  public ZeroIntakeCommand(boolean forced) {
    this.forced = forced;
    addRequirements(Intake.getInstance());
  }

  @Override
  public void initialize() {
    if (Intake.getInstance().getIsZeroed() && !forced) { // VERY IMPORTANT, DO NOT REMOVE
      // If the intake is already zeroed and we're not forcing,
      // just cancel this command
      shouldZero = false;
      this.cancel();
      return;
    }
    Intake.getInstance().setIsZeroed(false);
    Intake.getInstance().setZeroingVoltage();
    shouldZero = true;
  }

  @Override
  public boolean isFinished() {
    return Intake.getInstance().getVelocity() < Constants.Intake.ZERO_MIN_CURRENT;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && shouldZero) { // VERY IMPORTANT, DO NOT REMOVE
      Intake.getInstance().stop();
      Intake.getInstance().zero();
    }
  }
}
