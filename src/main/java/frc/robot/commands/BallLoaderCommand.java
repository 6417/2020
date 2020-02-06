/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;

public class BallLoaderCommand extends CommandBase {
  /**
   * Creates a new BallShooter_loaderCommand.
   */
  private BallShooterSubsystem m_subsystem = BallShooterSubsystem.getInstance();
  private DoubleSupplier loaderSpeed;

  public BallLoaderCommand(DoubleSupplier loaderSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loaderSpeed = loaderSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TestRobotContainer.getInstance().setLoadSliderPos(loaderSpeed.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.setLoader(loaderSpeed.getAsDouble());
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
