package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

public class PickUpCommand extends SequentialCommandGroup {
    public PickUpCommand() {
        super(new PneumaticPickupModuleCommand(PneumaticState.FORWARD), 
                new ParallelCommandGroup(new BallPickupMotorCommand(Constants.standardPickUpMotorSpeed),
                new TransportBallCommand(Constants.standardTransportSpeed, false))
            );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        new PneumaticPickupModuleCommand(PneumaticState.REVERSE).schedule();
    }
}