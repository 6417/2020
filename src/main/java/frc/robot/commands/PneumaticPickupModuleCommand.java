/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

public class PneumaticPickupModuleCommand extends CommandBase {
  /**
   * Creates a new ExtendPickupModule.
   */

  private PneumaticState state;
  
  public PneumaticPickupModuleCommand(PneumaticState state) {
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BallPickUpSubsystem.getInstance().setPickupCylinder(state);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
