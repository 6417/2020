/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

import java.lang.reflect.Executable;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PneumaticLiftCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final PneumaticSubsystem m_subsystem;
  private final ControlPanelSubsystem mPanelSubsystem = ControlPanelSubsystem.getInstance();
  private final PneumaticState state;
  private boolean isExtended;
  /**
   * Creates a new LiftCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PneumaticLiftCommand(PneumaticSubsystem subsystem, PneumaticState state) {
    m_subsystem = subsystem;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("lift command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("executing LiftCommand");
    switch (state) {
      case FORWARD:
        
        isExtended = m_subsystem.extendLift();          
        
        break;
        
      case REVERSE:
        if (!mPanelSubsystem.getReedBumperFront()){
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == PneumaticState.FORWARD) {
      return isExtended;
    } else {
      return mPanelSubsystem.getReedLiftBotom();
    }
  }
}
