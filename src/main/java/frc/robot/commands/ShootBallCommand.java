package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ShootBallCommand extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelCommand.
   */

  public ShootBallCommand() {
    super(new BallShooterCommand(Constants.standardShooterSpeed, true),
          new BallLoaderCommand(Constants.standardLoaderSpeed),
          new TransportBallCommand(Constants.standardTransportSpeed));
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    super(new BallShooterCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[0], true),
          new BallLoaderCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[2]),
          new TransportBallCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[1]));
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed, boolean speedEquaZero) {
    super(new BallShooterCommand(shooterSpeed, true), new TransportBallCommand(transportSpeed),
        new BallLoaderCommand(loaderSpeed));
  }

  private static double[] actualSpeed(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    double[] speeds = new double[3];
    if (shooterSpeed == 0) {
      speeds[0] = Constants.standardShooterSpeed;
    } else {
      speeds[0] = shooterSpeed;
    }

    if (loaderSpeed == 0) {
      speeds[1] = Constants.standardLoaderSpeed;
    } else {
      speeds[1] = loaderSpeed;
    }

    if (transportSpeed == 0) {
      speeds[2] = Constants.standardTransportSpeed;
    } else {
      speeds[2] = transportSpeed;
    }

    return speeds;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
