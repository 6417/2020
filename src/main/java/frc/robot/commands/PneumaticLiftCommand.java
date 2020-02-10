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
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

/**
 * An example command that uses an example subsystem.
 */
public class PneumaticLiftCommand extends CommandBase {
  private final ControlPanelSubsystem m_subsystem = ControlPanelSubsystem.getInstance();
  private final PneumaticState state;
  private boolean isExtended;
  /**
   * Creates a new LiftCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PneumaticLiftCommand(PneumaticState state) {
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case FORWARD:
        
        isExtended = m_subsystem.extendLift();          
        
        break;
        
      case REVERSE:
        if (!m_subsystem.getReedBumperFront()){
          m_subsystem.retractLift();
        }
        else {System.out.println("You can't retract the lift when the bumper is extended!");}

        break;
      default:
        System.out.println("State must be FORWARD or REVERSE not " + String.valueOf(state));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == PneumaticState.FORWARD) {      
      return true;
    } else {
      return m_subsystem.getReedLiftBotom();
    }
    // return true;
  }
}
