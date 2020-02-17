/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class SetMotorForRotationsCommand extends CommandBase {
  private final ControlPanelSubsystem m_subsystem = ControlPanelSubsystem.getInstance();
  private double rotations;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetMotorForRotationsCommand(double rotations) {
    this.rotations = rotations;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSensorPos(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_subsystem.getReedLiftBotom()){
      m_subsystem.setMotorForRotations(rotations);
  
    }
    else {System.out.println("You have to extend the cylinders first to rotate the Motor!");}    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    if(!m_subsystem.getReedLiftBotom()){
      return m_subsystem.isMotorInRnage();
    }
    else{return true;}
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
