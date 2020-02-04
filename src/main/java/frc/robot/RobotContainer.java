/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Set;

import com.fasterxml.jackson.databind.deser.std.PrimitiveArrayDeserializers;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendControlPanel_group;
import frc.robot.commands.RetractControlPanel_group;
import frc.robot.commands.SetMotorForRotationsCommand;
import frc.robot.commands.TransportBallCommand;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ShuffleBoardInformation controlPanelModuleConected;
  private String tab = "Informations";

  public static final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

  private static final JoystickButton loaderNoAutoMachanismButton = new JoystickButton(joystick, Constants.LOADER_NO_AUTOMECHANISMS_BUTTON_NUMBER);
  private static final JoystickButton transportMotorNoAutoMechanismsButton = new JoystickButton(joystick, Constants.TRANSPORT_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER);
  private static final JoystickButton controlPanelMotorNoAutoMechanismsButton = new JoystickButton(joystick, Constants.CONTROL_PANEL_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMPER);
  private static final JoystickButton shooterNoAutoMechanismsButton = new JoystickButton(joystick, Constants.SHOOTER_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER);

  private static final JoystickButton deacitvateSecurityMechanismsButton = new JoystickButton(joystick,
      Constants.DEACTIVATE_SECUTITY_MECHANISMS_BUTTON_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    showOnShuffleBoard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  public void configureButtonBindings() {
    loaderNoAutoMachanismButton.whileHeld(new BallLoaderCommand(Constants.standardLoaderSpeed));
    transportMotorNoAutoMechanismsButton.whileHeld(new TransportBallCommand(true));
    controlPanelMotorNoAutoMechanismsButton.whileHeld(new CommandBase(){
      @Override
      public void execute() {
        ControlPanelSubsystem.getInstance().setMotor(0.45);
      }

      @Override
      public void end(boolean interrupted) {
        ControlPanelSubsystem.getInstance().stopMotor();
      }
      
      @Override
        public boolean isFinished() {
          return false;
        }
    });

    shooterNoAutoMechanismsButton.whileHeld(new CommandBase() {
      @Override
      public void execute() {
        BallShooterSubsystem.getInstance().setShooter(Constants.standardShooterSpeed.getAsDouble());
      }

      @Override
      public void end(boolean interrupted) {
        BallShooterSubsystem.getInstance().stopShooter();
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    });
  }

  private void showOnShuffleBoard() {
    controlPanelModuleConected = new ShuffleBoardInformation(tab, "Control panel module conected",
        ControlPanelSubsystem.getInstance().isConected);
  }

  public void updateShuffleBoard() {
    controlPanelModuleConected.update(ControlPanelSubsystem.getInstance().isConected);
  }

  public static boolean getSecurityMechanismsButton() {
    return deacitvateSecurityMechanismsButton.get();
  }

  public static boolean getLoaderNoAutoMechanismsButton() {
    return loaderNoAutoMachanismButton.get();
  }
}
