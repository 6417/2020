/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;

public class BallLoaderCommand extends CommandBase {
  /**
   * Creates a new BallShooter_loaderCommand.
   */
  private static BallShooterSubsystem m_subsystem = BallShooterSubsystem.getInstance();
  private double loaderSpeed;

  public BallLoaderCommand(double loaderSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loaderSpeed = loaderSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TestRobotContainer.getInstance().setLoadSliderPos(loaderSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setLoader(loaderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return BallTransportSubsystem.getInstance().getSensor();
  }
}
