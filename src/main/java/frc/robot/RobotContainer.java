/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;
import com.kauailabs.navx.frc.AHRS;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.BallPickupMotorCommand;
import frc.robot.commands.ControlPanelPneumaticCommandGroup;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.TransportBallCommand;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

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
  private static RobotContainer mInstance;

  public static final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

  // no automechanisms Buttons

  private final JoystickButton loaderNoAutoMachanismButton = new JoystickButton(joystick,
      Constants.LOADER_NO_AUTOMECHANISMS_BUTTON_NUMBER);
  private final JoystickButton transportMotorNoAutoMechanismsButton = new JoystickButton(joystick,
      Constants.TRANSPORT_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER);
  private final JoystickButton controlPanelMotorNoAutoMechanismsButton = new JoystickButton(joystick,
      Constants.CONTROL_PANEL_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMPER);
  private final JoystickButton shooterNoAutoMechanismsButton = new JoystickButton(joystick,
      Constants.SHOOTER_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER);
  private final JoystickButton deacitvateSecurityMechanismsButton = new JoystickButton(joystick,
      Constants.DEACTIVATE_SECUTITY_MECHANISMS_BUTTON_NUMBER);
  
  // Initialize Buttons
  private final JoystickButton ballPickupMotorButton = new JoystickButton(joystick, Constants.BALL_PICKUP_MOTOR_BUTTON_NUMPER);
  private final JoystickButton shootBallButton = new JoystickButton(joystick, Constants.SHOOT_BUTTON_NUMBER);
  private final JoystickButton extendAndRetactControlPanelButton = new JoystickButton(joystick, Constants.EXTEND_AND_RETRACT_CONTROL_PANEL_MODULE_BUTTON_NUMBER);
  private final JoystickButton cancelAllCommandsButton = new JoystickButton(joystick, Constants.CANCEL_ALL_COMMANDS_BUTTON_NUMBER);

  // Initialize Commands
  private CommandBase ballLoaderCommand = new CommandBase() {
    @Override
    public void initialize() {
      BallShooterSubsystem.getInstance().setLoader(Constants.standardLoaderSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
      BallShooterSubsystem.getInstance().stopLoader();
    }

   @Override
   public boolean isFinished() {
    return false;
   } 
    
  };
  private TransportBallCommand noAutoMechsTransportBallCommand = new TransportBallCommand(true);
  private CommandBase noAutoMechsControlPanelMotorCommand = new CommandBase() {
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
  };

  private CommandBase noAutoMechsShooterCommand = new CommandBase() {
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
  };

  // private ConditionalCommand extendAndRetractControlPanelModuleCommand = new ConditionalCommand(new ControlPanelPneumaticCommandGroup(PneumaticState.FORWARD), 
  //     new ControlPanelPneumaticCommandGroup(PneumaticState.REVERSE), 
  //     () -> ControlPanelSubsystem.getInstance().getReedLiftBotom() || getSecurityMechanismsButton());

  private SequentialCommandGroup extendAndRetractControlPanelModuleCommand = new ControlPanelPneumaticCommandGroup(PneumaticState.FORWARD);

  private CommandBase cancelAllCommands = new CommandBase() {
    @Override
    public boolean isFinished() {
      CommandScheduler.getInstance().cancelAll();
      return true;
    }
  };

  private ParallelCommandGroup pickUpCommand = new ParallelCommandGroup(new BallPickupMotorCommand(Constants.standardPickUpMotorSpeed),
      new TransportBallCommand(Constants.standardTransportSpeed, false));

  public SequentialCommandGroup shootBallCommand = new ShootBallCommand();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    configureButtonBindings();
    showOnShuffleBoard();
  }

  public static RobotContainer getInstance() {
    if (mInstance == null) {
      mInstance = new RobotContainer();
    }
    return mInstance;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  public void configureButtonBindings() {
    //No auto mechanisms button bindings
    loaderNoAutoMachanismButton.whileHeld(ballLoaderCommand);
    transportMotorNoAutoMechanismsButton.whileHeld(noAutoMechsTransportBallCommand);
    controlPanelMotorNoAutoMechanismsButton.whileHeld(noAutoMechsControlPanelMotorCommand);
    shooterNoAutoMechanismsButton.whileHeld(noAutoMechsShooterCommand);
    //

    ballPickupMotorButton.whileHeld(pickUpCommand);

    shootBallButton.toggleWhenPressed(shootBallCommand);
    extendAndRetactControlPanelButton.toggleWhenPressed(extendAndRetractControlPanelModuleCommand);
    cancelAllCommandsButton.whenPressed(cancelAllCommands);

    // for the standart drive command
    DriveSubsystem.getInstance();
  }

  private void showOnShuffleBoard() {
    controlPanelModuleConected = new ShuffleBoardInformation(tab, "Control panel module conected", ControlPanelSubsystem.getInstance().isConected);
  }

  public void updateShuffleBoard() {
    controlPanelModuleConected.update(ControlPanelSubsystem.getInstance().isConected);
  }

  public static boolean getSecurityMechanismsButton() {
    return mInstance.deacitvateSecurityMechanismsButton.get();
  }

  public boolean getLoaderNoAutoMechanismsButton() {
    return loaderNoAutoMachanismButton.get();
  }
}