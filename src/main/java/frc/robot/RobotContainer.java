/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.SetMotorForRotationsCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public Joystick jostk = new Joystick(0);


  /**
   * Pneumatic Subsystem constants used in {@link PneumaticSubsystem}
   */

   public static final int PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID = 0;
   public static final int PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID = 0;
   public static final int PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID = 1;
   public static final int PNEUMATIC_SUBSYSTEM_BUMPER_SOLENOID_EXTEND_ID = 2;
   public static final int PNEUMATIC_SUBSYSTEM_BUMPER_SOLENOID_RETRACT_ID = 3;


  /**
   * Control Panel Subsystem constants used in {@link ControlPanelSubsystem}.
   */

  public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_BOTTOM_REED_DI_ID = 0;
  public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_TOP_REED_DI_ID = 1;
  public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_BACK_REED_DI_ID = 2;
  public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_FRONT_REED_DI_ID = 3;
  public static final I2C.Port CONTROL_PANEL_SUBSYSTEM_COLOR_SENSOR_I2C_PORT = I2C.Port.kOnboard;
  public static final int CONTROL_PANEL_SUBSYSTEM_MOTOR_CAN_ID = 48;

  public static JoystickButton pneumaticLiftButtonExtend;
  public static JoystickButton pneumaticLiftButtonReject;
  public static JoystickButton pneumaticBumperButtonExtend;
  public static JoystickButton pneumaticBumperButtonReject;
  public static JoystickButton setMotorForRotationsButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    pneumaticLiftButtonExtend = new JoystickButton(jostk, 1);
    pneumaticLiftButtonReject = new JoystickButton(jostk, 2);
    pneumaticBumperButtonExtend = new JoystickButton(jostk, 3);
    pneumaticBumperButtonReject = new JoystickButton(jostk, 4);
    setMotorForRotationsButton = new JoystickButton(jostk, 5);

    pneumaticLiftButtonExtend.whenPressed(new PneumaticLiftCommand(Robot.pneumaticSubsystem, PneumaticState.FORWARD));
    pneumaticLiftButtonReject.whenPressed(new PneumaticLiftCommand(Robot.pneumaticSubsystem, PneumaticState.REVERSE));
    pneumaticBumperButtonExtend.whenPressed(new PneumaticBumperCommand(Robot.pneumaticSubsystem, PneumaticState.FORWARD));
    pneumaticBumperButtonReject.whenPressed(new PneumaticBumperCommand(Robot.pneumaticSubsystem, PneumaticState.REVERSE));
    setMotorForRotationsButton.whenPressed(new SetMotorForRotationsCommand(Robot.controlPanelSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
