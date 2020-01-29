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
import frc.robot.subsystems.BallTransportSubsystem;

public class TransportBallCommand extends CommandBase {
  /**
   * Creates a new BallTransportCommand.
   */
  private double speed;
  private BallTransportSubsystem m_subsystem = BallTransportSubsystem.getInstance();
  private LatchedBoolean finished;
  private boolean stop = false;

  public TransportBallCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  }

  public TransportBallCommand(boolean stop) {
    this.speed = 0;
    this.stop = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = new LatchedBoolean(EdgeDetection.FALLING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.setTransportMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished.update(m_subsystem.getSensor()) || stop;
  }
}
