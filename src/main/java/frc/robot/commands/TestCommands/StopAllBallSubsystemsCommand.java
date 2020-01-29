package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.TestRobotContainer;
import frc.robot.commands.BallTransportCommand;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;

public class StopAllBallSubsystemsCommand extends CommandBase {
    private static BallShooterSubsystem mSubsystem = BallShooterSubsystem.getInstance();
    private static BallTransportSubsystem mTransportSubsystem = BallTransportSubsystem.getInstance();
    private static BallPickUpSubsystem mPickUpSubsystem = BallPickUpSubsystem.getInstance();

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
      mPickUpSubsystem.stopPickUpMotor();
      mTransportSubsystem.stopTransportMotor();

      TestRobotContainer.getInstance().setLoadSliderPos(0);
      TestRobotContainer.getInstance().setShooterSliderPos(0);
      TestRobotContainer.getInstance().setTransportSliderPos(0);     
      TestRobotContainer.getInstance().setPickUpSliderPos(0);
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