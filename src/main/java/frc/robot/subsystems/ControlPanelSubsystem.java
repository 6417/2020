/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControlPanelSubsystem extends SubsystemBase {
  private static ControlPanelSubsystem mInstance;
  private final int ticksPerRotation = 10000;
  private final int range = 100;

  private enum SystemState {
    RETRACTED, EXTENDED, CONTACTED, RETRACTING, EXTENDING
  }

  public enum ColorDetected {
    RED, GREEN, BLUE, YELLOW, NONE
  }

  // The system assumes that the valve is not at homed position
  // private SystemState state = SystemState.RETRACTING;

  private ColorSensorV3 colorSensor = new ColorSensorV3(Constants.CONTROL_PANEL_SUBSYSTEM_COLOR_SENSOR_I2C_PORT);
  public WPI_TalonSRX motor = new WPI_TalonSRX(Constants.CONTROL_PANEL_SUBSYSTEM_MOTOR_CAN_ID);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private int rotations;

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.43, 0.39, 0.16);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * Creates a new ControlPanelSubsystem.
   */
  private ControlPanelSubsystem() {
    SendableRegistry.addChild(this, motor);
    SendableRegistry.setName(motor, "Control Panel Motor");
    configMotor();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.setConfidenceThreshold(0.95);
  }

  private void configMotor() {
    // motor.configFactoryDefault();
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    /* Configure Sensor Source for Pirmary PID */
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
    motor.selectProfileSlot(0, 0);
		motor.config_kF(0, 0, 20);
		motor.config_kP(0, 0, 20);
		motor.config_kI(0, 0, 20);
		motor.config_kD(0, 0, 20);
  }

  public static ControlPanelSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ControlPanelSubsystem();
    }
    return mInstance;
  }

  public ColorDetected getColor() {
    ColorMatchResult match = m_colorMatcher.matchColor(colorSensor.getColor());
    ColorDetected color = ColorDetected.NONE;
    if (match == null) {
      return ColorDetected.NONE;
    } else if (match.color == kBlueTarget) {
      color = ColorDetected.BLUE;
    } else if (match.color == kRedTarget) {
      color = ColorDetected.RED;
    } else if (match.color == kGreenTarget) {
      color = ColorDetected.GREEN;
    } else if (match.color == kYellowTarget) {
      color = ColorDetected.YELLOW;
    }
    return color;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getReedLiftBotom() {
    return motor.isRevLimitSwitchClosed() == 1;
  }

  public boolean getReedBumperFront() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  public void setSensorPos(int pos) {
    motor.setSelectedSensorPosition(pos);
  }

  public void setMotorForRotations(int rotations) {
    this.rotations = rotations;
    if (motor.getSelectedSensorPosition() < this.rotations * ticksPerRotation - range) {
      motor.set(0.2);
    } else if (motor.getSelectedSensorPosition() > this.rotations * ticksPerRotation + range) {
      motor.set(-0.2);
    } 
  }

  public boolean isMotorInRnage() {
    if (motor.getSelectedSensorPosition() > rotations * ticksPerRotation - range && motor.getSelectedSensorPosition() < rotations * ticksPerRotation + range) {
      motor.stopMotor();
      return true;
    } else {
      return false;
    }
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Color", () -> this.getColor().toString(), null);
  }
}
