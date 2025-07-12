package org.chillout1778.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.chillout1778.subsystems.Arm;
import org.chillout1778.subsystems.Superstructure;

public class TeleopTesterCommand extends Command {

  public TeleopTesterCommand() {
    // addRequirements(Arm.getInstance());
  }

  @Override
  public void execute() {
    // Arm.getInstance().setState(Arm.PivotState.GetAlgae, Arm.RollerState.Idle);
  }

  @Override
  public void end(boolean interrupted) {
    // Superstructure.getInstance().emptyInputs();
  }
}