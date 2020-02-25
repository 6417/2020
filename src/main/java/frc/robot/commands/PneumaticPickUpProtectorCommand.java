package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

public class PneumaticPickUpProtectorCommand extends CommandBase {
    private PneumaticState state;

    public PneumaticPickUpProtectorCommand() {
        state = PneumaticState.OFF;
    }

    public PneumaticPickUpProtectorCommand(boolean state) {
        if (state) {
            this.state = PneumaticState.FORWARD;
        } else {
            this.state = PneumaticState.REVERSE;
        }
    }

    @Override
    public void initialize() {
        if (!BallPickUpSubsystem.getInstance().protectorExtended && state == PneumaticState.OFF) {
            BallPickUpSubsystem.getInstance().setProtectCylinder(PneumaticState.FORWARD);
            BallPickUpSubsystem.getInstance().protectorExtended = true;
        } else if (state == PneumaticState.OFF) {
            BallPickUpSubsystem.getInstance().setProtectCylinder(PneumaticState.REVERSE);
            BallPickUpSubsystem.getInstance().protectorExtended = false;
        } else if (state != PneumaticState.OFF) {
            BallPickUpSubsystem.getInstance().setProtectCylinder(state);
            BallPickUpSubsystem.getInstance().protectorExtended = (state == PneumaticState.FORWARD);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}