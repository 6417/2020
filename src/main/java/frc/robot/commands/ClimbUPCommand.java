/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class ClimbUPCommand extends CommandBase {
  /**
   * Creates a new ClimbUPCommand.
   */
  public ClimbUPCommand() {
    addRequirements(DriveSubsystem.getInstance());
  }

  public ClimbUPCommand(ClimberSubsystem mClimberSubsystem) {
    addRequirements(mClimberSubsystem);
  }

  // Called when the command is initially scheduled.

  @Override
  public void execute() {
    ClimberSubsystem.getInstance().climb();
  }

  @Override
  public void end(boolean interrupted) {
    ClimberSubsystem.getInstance().stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
