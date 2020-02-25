/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ControlPanelMotorCommand extends CommandBase {
  /**
   * Creates a new setControlPanelMotorCommand.
   */
  private ControlPanelSubsystem mSubsystem;
  private DoubleSupplier speed;
  public ControlPanelMotorCommand(DoubleSupplier speed) {
    this.speed = speed;
    mSubsystem = ControlPanelSubsystem.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TestRobotContainer.getInstance().setControlPanelMotorSlider(speed.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSubsystem.setMotor(speed.getAsDouble());
    System.out.println("ControlPanelMotorCommand initialized");
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
