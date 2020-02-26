package frc.robot.commands.AutonomusCommads;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveOverLineCommand extends CommandBase {
    public DriveOverLineCommand() {

    }

    @Override
    public void execute() {
        DriveSubsystem.getInstance().drive(0.25, 0);
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().stopDrive();
    }

    @Override
    public boolean isFinished() {
        Pose2d pos = DriveSubsystem.getInstance().getPose();
        return pos.getTranslation().getX() >= Constants.DISTANCE_TO_DRIVE_OVER_LINE;
    }
}