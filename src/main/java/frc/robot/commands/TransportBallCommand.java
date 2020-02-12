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
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.BallTransportSubsystem;

public class TransportBallCommand extends CommandBase {
  /**
   * Creates a new BallTransportCommand.
   */
  private DoubleSupplier speed;
  private BallTransportSubsystem m_subsystem = BallTransportSubsystem.getInstance();
  private LatchedBoolean finished;
  private boolean shoot = false;
  private boolean automechanismsDisabled = false;

  public TransportBallCommand(DoubleSupplier speed, boolean shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = shoot;
    this.speed = speed;
  }

  public TransportBallCommand(boolean automechanismsDisabled) {
    this.automechanismsDisabled = automechanismsDisabled;
    speed = Constants.standardTransportSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (speed.getAsDouble() == 0) {
      this.speed = () -> 0.25;
    }

    if (shoot) {
      finished = new LatchedBoolean(EdgeDetection.FALLING);
    } else {
      finished = new LatchedBoolean(EdgeDetection.RISING);
    }
    TestRobotContainer.getInstance().setTransportSliderPos(this.speed.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.setTransportMotor(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_subsystem.stopTransportMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shoot) {
      return finished.update(m_subsystem.getSensor());
    } else if (automechanismsDisabled) {
      return false;
    } else {
      return finished.update(m_subsystem.getSensor()) || m_subsystem.getSensor();
    }
  }
}