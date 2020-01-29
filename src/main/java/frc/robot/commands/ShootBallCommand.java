package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootBallCommand extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelCommand.
   */

  public ShootBallCommand() {
    super(new TransportBallCommand(0.25) ,new BallShooterCommand(0.9, true), new BallLoaderCommand(0.25));
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    super(new TransportBallCommand(transportSpeed) ,new BallShooterCommand(shooterSpeed, true), new BallLoaderCommand(loaderSpeed));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
