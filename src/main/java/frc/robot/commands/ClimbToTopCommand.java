package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbToTopCommand extends CommandBase {
    public ClimbToTopCommand() {
        addRequirements(ClimberSubsystem.getInstance());
    }

    @Override
    public void execute() {
        ClimberSubsystem.getInstance().climb(Constants.standardClimberSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.getInstance().stopClimber();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(ClimberSubsystem.getInstance().getHeight()) >= Math.abs(Constants.CLIMBER_TOP_POSITION);
    }
}