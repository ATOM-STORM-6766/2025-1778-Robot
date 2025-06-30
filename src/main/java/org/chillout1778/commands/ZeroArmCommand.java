package org.chillout1778.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.chillout1778.subsystems.Arm;

public class ZeroArmCommand extends Command {
    private final boolean forced;

    public ZeroArmCommand() {
        this(false);
    }

    public ZeroArmCommand(boolean forced) {
        this.forced = forced;
        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        if (Arm.getInstance().isZeroed && !forced) { // VERY IMPORTANT, DO NOT REMOVE
            this.cancel();
            return;
        }
        Arm.getInstance().resetRelativeFromAbsolute();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
