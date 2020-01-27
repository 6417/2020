package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class StopTankDriveCommand extends CommandBase {
    private static DriveSubsystem mSubsystem = DriveSubsystem.getInstance();

    public StopTankDriveCommand() {
    }

          // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mSubsystem.stopDrive();
      TestRobotContainer.setDriveSlider(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}