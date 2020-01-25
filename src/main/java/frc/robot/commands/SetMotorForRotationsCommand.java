/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ControlPanelSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class SetMotorForRotationsCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ControlPanelSubsystem m_subsystem;
  private int rotations;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetMotorForRotationsCommand(ControlPanelSubsystem subsystem, int rotations) {
    m_subsystem = subsystem;
    this.rotations = rotations;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
