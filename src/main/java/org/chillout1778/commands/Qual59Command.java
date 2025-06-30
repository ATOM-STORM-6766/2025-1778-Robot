package org.chillout1778.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.subsystems.Arm;
import org.chillout1778.subsystems.Elevator;
import org.chillout1778.subsystems.Superstructure;

public class Qual59Command extends Command {
    /*
                225  135  L4ScoreCoral
       212.28   260  100  L4FinishScoreCoral
       212.70   0    0    Down
       213.24   250  110  DescoreAlgae
     */
    
    private final Timer timer = new Timer();
    private int phase = 0;

    public Qual59Command() {
        addRequirements(Superstructure.getInstance(), Arm.getInstance(), Elevator.getInstance());
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
        Elevator.getInstance().setState(Elevator.State.Barge);
        phase = 0;
    }

    @Override
    public void execute() {
        if (Elevator.getInstance().state != Elevator.State.Barge || !Elevator.getInstance().getAtSetpoint()) {
            return; // unnecessary safeties
        }
        
        switch (phase) {            case 0:
                if (Elevator.getInstance().getAtSetpoint()) {
                    phase = 1;
                    // Set arm pivot state when Arm is complete
                    Arm.getInstance().setState(Arm.PivotState.ScoreCoral, Arm.RollerState.SlowOut);
                }
                break;            case 1:
                if (Arm.getInstance().getAtSetpoint()) {
                    phase = 2;
                    Arm.getInstance().setState(Arm.PivotState.L4FinishScoreCoral, Arm.RollerState.Out);
                    timer.restart();
                }
                break;
            case 2:
                if (timer.hasElapsed(0.42)) {
                    phase = 3;
                    Arm.getInstance().setState(Arm.PivotState.Down, Arm.RollerState.Off);
                }
                break;
            case 3:
                if (timer.hasElapsed(0.96)) {
                    phase = 4;
                    Arm.getInstance().setState(Arm.PivotState.DescoreAlgae, Arm.RollerState.Descore);
                }
                break;
        }
    }    @Override
    public void end(boolean interrupted) {
        // Reset superstructure inputs when command ends
        Superstructure.getInstance().emptyInputs();
    }
}
