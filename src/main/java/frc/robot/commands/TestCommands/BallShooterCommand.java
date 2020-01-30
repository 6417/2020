/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.BallShooterSubsystem;

public class BallShooterCommand extends CommandBase {
  /**
   * Creates a new BallShooterCommand.
   */
  private static BallShooterSubsystem m_subsystem = BallShooterSubsystem.getInstance();
  private double shooterSpeed;
  private boolean FixedSpeed;

  public BallShooterCommand(double shooterSpeed, boolean FixedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSpeed = shooterSpeed;
    this.FixedSpeed = FixedSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TestRobotContainer.getInstance().setShooterSliderPos(shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setShooter(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (FixedSpeed){
      if (m_subsystem.getSpeed() > 5400){
         return true;
      } else {
        return false;
      }
    }
    else{
      return true;
    }
  }
}