/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;
import lombok.Getter;

public class ControlPanelSubsystem extends SubsystemBase {

  private static ControlPanelSubsystem mInstance;

  private enum SystemState {
    RETRACTED, EXTENDED, CONTACTED, RETRACTING, EXTENDING
  }

  // The system assumes that the valve is not at homed position
  private SystemState state = SystemState.RETRACTING;

  private ColorSensorV3 colorSensor = new ColorSensorV3(RobotContainer.CONTROL_PANEL_SUBSYSTEM_COLOR_SENSOR_I2C_PORT);
  private WPI_TalonSRX motor = new WPI_TalonSRX(RobotContainer.CONTROL_PANEL_SUBSYSTEM_MOTOR_CAN_ID);

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {
    
  }

  public static ControlPanelSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ControlPanelSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getReedLiftTop() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  public boolean getReedBumperFront() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }
}
