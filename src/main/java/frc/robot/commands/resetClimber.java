/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class resetClimber extends CommandBase {
  /**
   * Creates a new resetClimber.
   */
  private ClimberSubsystem mSubsystem;
  private double rightSpeed;
  private double leftSpeed;

  public resetClimber() {
    mSubsystem = ClimberSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightSpeed = -0.1;
    leftSpeed = -0.1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSubsystem.setClimberLeft(leftSpeed);
    mSubsystem.setClimberRight(rightSpeed);
    if(mSubsystem.getRightLimit()) {
      rightSpeed = 0;
    }
    if(mSubsystem.getLeftLimit()) {
      leftSpeed = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mSubsystem.getLeftLimit() && mSubsystem.getRightLimit();
  }
}
