/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPickUpSubsystem;

public class BallPickupMotorCommand extends CommandBase {
  /**
   * Creates a new BallPickupCommand.
   */
  private static BallPickUpSubsystem m_subsytem = BallPickUpSubsystem.getInstance();
  private DoubleSupplier speed;
  public BallPickupMotorCommand(DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsytem.setPickUpMotor(speed.getAsDouble());
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
