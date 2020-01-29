/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyControlPanelSubsystem;

public class ControlPanelSubsystem extends SubsystemBase {
  private static ControlPanelSubsystem mInstance;
  private final int ticksPerRotation = 4096;
  private final int range = 25;

  private DoubleSolenoid liftSolenoid;
  private DoubleSolenoid bumperSolenoid;

  public enum ColorDetected {
    RED, GREEN, BLUE, YELLOW, NONE
  }

  // The system assumes that the valve is not at homed position
  // private SystemState state = SystemState.RETRACTING;

  
  private ColorSensorV3 colorSensor;
  private WPI_TalonSRX motor;

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

  public enum PneumaticState {
    OFF, FORWARD, REVERSE
  }

  /**
   * Creates a new ControlPanelSubsystem.
   */
  protected ControlPanelSubsystem() {
    constructor();
  }

  protected void constructor() {
    colorSensor = new ColorSensorV3(Constants.CONTROL_PANEL_SUBSYSTEM_COLOR_SENSOR_I2C_PORT);
    motor = new WPI_TalonSRX(Constants.CONTROL_PANEL_SUBSYSTEM_MOTOR_CAN_ID);

    addChild("Control Panel Motor", motor);

    motor.configFactoryDefault();
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.setConfidenceThreshold(0.95);

    liftSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
    Constants.CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID,
    Constants.CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID);

    bumperSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
        Constants.CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_EXTEND_ID,
        Constants.CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_RETRACT_ID);

    SendableRegistry.addChild(this, liftSolenoid);
    SendableRegistry.addChild(this, bumperSolenoid);
    // SendableRegistry.addChild(this, compressor);
    SendableRegistry.setName(liftSolenoid, "Lift");
    SendableRegistry.setName(bumperSolenoid, "Bumper");
  }

  public static ControlPanelSubsystem getInstance() {
    if (Constants.CONTROL_PANEL_SUBSYSTEM_ENABLED) {
      if (mInstance == null) {
         mInstance = new ControlPanelSubsystem();
      }
      return mInstance;
    } else {
      return new EmptyControlPanelSubsystem();
    }
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
    if (motor.getSelectedSensorPosition() < (this.rotations * ticksPerRotation) - range) {
      motor.set(0.2);
    } else if (motor.getSelectedSensorPosition() > (this.rotations * ticksPerRotation) + range) {
      motor.set(-0.2);
    } 
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }

  public int getEncoderValue() {
    return motor.getSelectedSensorPosition();
  }

  public boolean isMotorInRnage() {
    if (motor.getSelectedSensorPosition() > (rotations * ticksPerRotation) - range && motor.getSelectedSensorPosition() < (rotations * ticksPerRotation) + range) {
      motor.stopMotor();
      return true;
    } else {
      return false;
    }
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  private void set(DoubleSolenoid cylinder, PneumaticState state) {
    switch (state) {
    case OFF:
      cylinder.set(Value.kOff);
      break;

    case FORWARD:
      cylinder.set(Value.kForward);
      break;

    case REVERSE:
      cylinder.set(Value.kReverse);
      break;
    }
  }

  public boolean extendLift() {
    set(liftSolenoid, PneumaticState.FORWARD);
//    set(liftSolenoid, PneumaticState.OFF);
    return true;
  }

  public void retractLift() {
    set(liftSolenoid, PneumaticState.REVERSE);
//    set(liftSolenoid, PneumaticState.OFF);
  }

  public void closeLift() {
    set(liftSolenoid, PneumaticState.OFF);
  }

  public void extendBumper() {
    set(bumperSolenoid, PneumaticState.FORWARD);
  }

  public boolean retractBumper() {
    set(bumperSolenoid, PneumaticState.REVERSE);
//    set(bumperSolenoid, PneumaticState.OFF);
    return true;
  }

  public boolean closeBumper() {
    set(bumperSolenoid, PneumaticState.OFF);
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Color", () -> this.getColor().toString(), null);
    builder.addDoubleProperty("Encoder", () -> this.getEncoderValue(), (double pos) -> this.motor.setSelectedSensorPosition((int)pos));
  }
}