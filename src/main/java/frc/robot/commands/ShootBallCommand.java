package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;

public class ShootBallCommand extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelCommand.
   */

  public ShootBallCommand() {
    super(new BallShooterCommand(Constants.standardShooterSpeed, true),
          new BallLoaderCommand(Constants.standardLoaderSpeed),
          new TransportBallCommand(Constants.standardTransportSpeed, false)); 
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    super(new BallShooterCommand(() -> shooterSpeed, true),
          new BallLoaderCommand(() -> loaderSpeed),
          new TransportBallCommand(() -> transportSpeed, true),
          new CommandBase() {
            @Override
            public void execute() {
              BallShooterSubsystem.getInstance().stopShooter();
              BallShooterSubsystem.getInstance().stopLoader();
              BallTransportSubsystem.getInstance().stopTransportMotor();
            }

            @Override
            public boolean isFinished() {
              return true;
            }
          });
  }

  public ShootBallCommand(DoubleSupplier shooterSpeed, DoubleSupplier loaderSpeed, DoubleSupplier transportSpeed) {
    super(new BallShooterCommand(shooterSpeed, true),
          new BallLoaderCommand(loaderSpeed),
          new TransportBallCommand(transportSpeed, true), 
          new CommandBase() {
            @Override
            public void execute() {
              BallShooterSubsystem.getInstance().stopShooter();
              BallShooterSubsystem.getInstance().stopLoader();
              BallTransportSubsystem.getInstance().stopTransportMotor();
            }

            @Override
            public boolean isFinished() {
              return true;
            }
          });
        }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
