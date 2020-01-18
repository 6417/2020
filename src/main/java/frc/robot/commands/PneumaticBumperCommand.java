/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;
import frc.robot.subsystems.ControlPanelSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PneumaticBumperCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PneumaticSubsystem m_subsystem;
  private final ControlPanelSubsystem m_controlPanelSubsystem = ControlPanelSubsystem.getInstance();
  private final PneumaticState state;
  private boolean isRetracted;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PneumaticBumperCommand(PneumaticSubsystem subsystem, PneumaticState state) {
    m_subsystem = subsystem;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      switch(state){
        case FORWARD:
            if ( m_controlPanelSubsystem.getReedLiftTop()) {
                m_subsystem.extendBumper();
                break;
            }
            else {
                System.out.println("The lift cylider must be in the top position to extend the Bumper");
            }
            
        case REVERSE:
            isRetracted = m_subsystem.retractBumper();
            break;
        default:
            System.out.println("State must be FORWARD or REVERSE not " + String.valueOf(state));
        
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == PneumaticState.FORWARD){
        return m_controlPanelSubsystem.getReedBumperFront();
    }
    else{
        return isRetracted;
    }
  }
}
