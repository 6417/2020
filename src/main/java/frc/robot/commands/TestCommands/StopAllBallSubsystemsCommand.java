package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.TestRobotContainer;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.BallPickupMotorCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.TransportBallCommand;

public class StopAllBallSubsystemsCommand extends ParallelCommandGroup {
  public StopAllBallSubsystemsCommand() {
    super(
        new TransportBallCommand(true), 
        new BallLoaderCommand(() -> 0), 
        new BallShooterCommand(() -> 0, false),
        new BallPickupMotorCommand(() -> 0), 
        new CommandBase() {
          @Override
          public void initialize() {
            TestRobotContainer.getInstance().setLoadSliderPos(0);
            TestRobotContainer.getInstance().setShooterSliderPos(0);
            TestRobotContainer.getInstance().setTransportSliderPos(0);
            TestRobotContainer.getInstance().setPickUpSliderPos(0);
          }

          @Override
            public boolean isFinished() {
              return true;
            }
        });
  }

}