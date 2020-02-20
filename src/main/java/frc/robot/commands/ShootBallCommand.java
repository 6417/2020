package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;
import frc.robot.commands.BallShooterCommand;

public class ShootBallCommand extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelCommand.
   */

  private static BallShooterCommand shooterCommand = new BallShooterCommand(Constants.standardShooterSpeed, true);
  private static BallLoaderCommand loaderCommand = new BallLoaderCommand(Constants.standardLoaderSpeed);
  private static TransportBallCommand transportBallCommand = new TransportBallCommand(Constants.standardTransportSpeed, false);

  public ShootBallCommand() {
    super(shooterCommand,
          loaderCommand,
          transportBallCommand);
  }

  public ShootBallCommand(double shooterSpeed, double loaderSpeed, double transportSpeed) {
    super(new BallShooterCommand(() -> shooterSpeed, true),
          new BallLoaderCommand(() -> loaderSpeed),
          new TransportBallCommand(() -> transportSpeed, true)
    );
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
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (interrupted) {
      BallShooterSubsystem.getInstance().stopShooter();
      BallShooterSubsystem.getInstance().stopLoader();
      BallTransportSubsystem.getInstance().stopTransportMotor();
      CommandScheduler.getInstance().cancel(shooterCommand, loaderCommand, transportBallCommand);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
