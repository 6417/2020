/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class DriveCommand extends CommandBase {
  DoubleSupplier xSpeed, ySpeed;
  boolean fixSpeed = false;

  /**
   * Creates a new DriveCommand.
   */
  private DriveSubsystem m_subsystem = DriveSubsystem.getInstance();
  public DriveCommand() {
    addRequirements(m_subsystem);
  }

  public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    this();
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.fixSpeed = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fixSpeed) {
      m_subsystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble());
    }
    else {
      m_subsystem.drive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
