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
    mSubsystem.stopDrive();
    TestRobotContainer.getInstance().setDriveSlider(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}