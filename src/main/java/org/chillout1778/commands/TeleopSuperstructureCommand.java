package org.chillout1778.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.Controls;
import org.chillout1778.subsystems.Superstructure;

public class TeleopSuperstructureCommand extends Command {

  public TeleopSuperstructureCommand() {
    addRequirements(Superstructure.getInstance());
  }

  @Override
  public void execute() {
    Superstructure.getInstance().setInputs(Controls.superstructureInputs());
  }

  @Override
  public void end(boolean interrupted) {
    Superstructure.getInstance().emptyInputs();
  }
}
