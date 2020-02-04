package frc.robot.commands;

import java.util.function.DoubleSupplier;

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
          new TransportBallCommand(Constants.standardTransportSpeed, true));
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    super(new BallShooterCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[0], true),
          new BallLoaderCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[2]),
          new TransportBallCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed)[1], true));
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed, boolean speedEqualZero) {
    super(new BallShooterCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed, speedEqualZero)[0], true),
          new BallLoaderCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed, speedEqualZero)[2]),
          new TransportBallCommand(actualSpeed(shooterSpeed, loaderSpeed, transportSpeed, speedEqualZero)[1], true));
  }

  public ShootBallCommand(DoubleSupplier shooterSpeed, DoubleSupplier loaderSpeed, DoubleSupplier transportSpeed) {
    super(new BallShooterCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble())[0], true),
          new BallLoaderCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble())[2]),
          new TransportBallCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble())[1], true));
  }

  public ShootBallCommand(DoubleSupplier shooterSpeed, DoubleSupplier loaderSpeed, DoubleSupplier transportSpeed, boolean speedEqualZero) {
    super(new BallShooterCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble(), speedEqualZero)[0], true),
          new BallLoaderCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble(), speedEqualZero)[2]),
          new TransportBallCommand(actualSpeed(shooterSpeed.getAsDouble(), loaderSpeed.getAsDouble(), transportSpeed.getAsDouble(), speedEqualZero)[1], true));
  }



  private static DoubleSupplier[] actualSpeed(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    DoubleSupplier[] speeds = new DoubleSupplier[3];
    if (shooterSpeed == 0) {
      speeds[0] = Constants.standardShooterSpeed;
    } else {
      speeds[0] = () -> shooterSpeed;
    }

    if (loaderSpeed == 0) {
      speeds[1] = Constants.standardLoaderSpeed;
    } else {
      speeds[1] = () -> loaderSpeed;
    }

    if (transportSpeed == 0) {
      speeds[2] = Constants.standardTransportSpeed;
    } else {
      speeds[2] = () -> transportSpeed;
    }

    return speeds;
  }

  private static DoubleSupplier[] actualSpeed(double shooterSpeed, double loaderSpeed, double transportSpeed, boolean speedEqualZero) {
    if (speedEqualZero) {
      DoubleSupplier[] speed = {() -> shooterSpeed, () -> loaderSpeed, () -> transportSpeed};
      return speed;
    } else {
      return actualSpeed(shooterSpeed, loaderSpeed, transportSpeed);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
