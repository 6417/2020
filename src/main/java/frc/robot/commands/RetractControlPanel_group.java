/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RetractControlPanel_group extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelCommand.
   */

  public RetractControlPanel_group(PneumaticSubsystem m_subsystem) {
    super(new PneumaticLiftCommand(m_subsystem, PneumaticState.FORWARD), new PneumaticBumperCommand(m_subsystem, PneumaticState.FORWARD));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }
}
