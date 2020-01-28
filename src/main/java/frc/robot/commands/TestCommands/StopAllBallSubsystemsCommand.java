package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.BallShooterSubsystem;

public class StopAllBallSubsystemsCommand extends CommandBase {
    private static BallShooterSubsystem mSubsystem = BallShooterSubsystem.getInstance();

    public StopAllBallSubsystemsCommand() {
    }

          // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the comPmand is scheduled.
  @Override
  public void execute() {
      mSubsystem.stopLoader();
      mSubsystem.stopShooter();
      TestRobotContainer.getInstance().setLoadSliderPos(0);
      TestRobotContainer.getInstance().setShooterSliderPos(0);
      TestRobotContainer.getInstance().setTransportSliderPos(0);
      TestRobotContainer.getInstance().transportMotor.set(0);
      
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