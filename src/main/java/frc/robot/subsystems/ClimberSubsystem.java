/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ch.team6417.lib.utils.Algorithms;
import ch.team6417.lib.utils.FridoCANSparkMax;
import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.emptySubsystems.EmptyClimberSubsystem;

public class ClimberSubsystem extends SubsystemBase {
  private static FridoCANSparkMax climber_motor_left;
  private static FridoCANSparkMax climber_motor_right;

  protected static CANPIDController climberPIDLeft;
  protected static CANPIDController climberPIDRight;

  private static CANEncoder climber_encoder_right;
  private static CANEncoder climber_encoder_left;

  private static CANDigitalInput left_limit;
  private static CANDigitalInput right_limit;

  private double positionDifference;

  public static ClimberSubsystem mInstance;
  private static LatchedBoolean firstGetInstatce = new LatchedBoolean(EdgeDetection.RISING);

  /**
   * Creates a new ClimberSubsystem.
   */
  protected ClimberSubsystem() {
    constructor();
  }

  protected void constructor() {
    if (firstGetInstatce.update(true)) {
      climber_motor_left = new FridoCANSparkMax(Constants.MOTOR_CLIMBER_LEFT_ID, MotorType.kBrushless);
      climber_motor_right = new FridoCANSparkMax(Constants.MOTOR_CLIMBER_RIGHT_ID, MotorType.kBrushless);

      climber_motor_left.restoreFactoryDefaults();
      climber_motor_right.restoreFactoryDefaults();

      climber_motor_left.setIdleMode(IdleMode.kBrake);
      climber_motor_right.setIdleMode(IdleMode.kBrake);

      climberPIDLeft = climber_motor_left.getPIDController();
      climberPIDRight = climber_motor_right.getPIDController();

      climber_encoder_left = climber_motor_left.getEncoder();
      climber_encoder_right = climber_motor_right.getEncoder();
      climber_encoder_left.setPosition(0);
      climber_encoder_right.setPosition(0);

      // set PID coefficients
      climberPIDLeft.setP(Constants.kPclimber);
      climberPIDLeft.setI(Constants.kIclimber);
      climberPIDLeft.setD(Constants.kDclimber);
      climberPIDLeft.setIZone(Constants.kIzclimber);
      climberPIDLeft.setFF(Constants.kFFclimber);
      climberPIDLeft.setOutputRange(Constants.kMinOutputclimber, Constants.kMaxOutputclimber);

      // set PID coefficients
      climberPIDRight.setP(Constants.kPclimber);
      climberPIDRight.setI(Constants.kIclimber);
      climberPIDRight.setD(Constants.kDclimber);
      climberPIDRight.setIZone(Constants.kIzclimber);
      climberPIDRight.setFF(Constants.kFFclimber);
      climberPIDRight.setOutputRange(Constants.kMinOutputclimber, Constants.kMaxOutputclimber);


      //Config Limits
      left_limit = climber_motor_left.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      right_limit = climber_motor_right.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);

      left_limit.enableLimitSwitch(true);
      right_limit.enableLimitSwitch(true);
    }

    super.addChild("Climber Motor Right", climber_motor_right);
    super.addChild("Climber Motor Left", climber_motor_left);
    positionDifference = 0;
  }

  public static ClimberSubsystem getInstance() {
    if (Constants.CLIMBER_SUBSYSTEM_ENABLED && mInstance == null) {
      mInstance = new ClimberSubsystem();
    } else if (mInstance == null) {
      mInstance = new EmptyClimberSubsystem();
    }
    firstGetInstatce.update(true);
    return mInstance;
  }



  @Override
  public void periodic() {

    checkSafetyStop();
    calculatePositionDifference();
    checkLimits();

    SmartDashboard.putNumber("Position Left", climber_motor_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Position Right", climber_motor_right.getEncoder().getPosition());
    SmartDashboard.putNumber("Position Difference", positionDifference);
    SmartDashboard.putNumber("Soll speed right", (-RobotContainer.getInstance().joystick.getY() * 0.5 + positionDifference) * 5700);
    SmartDashboard.putNumber("Soll speed left", (-RobotContainer.getInstance().joystick.getY() * 0.5 - positionDifference) * 5700);
    SmartDashboard.putBoolean("limit Right", right_limit.get());
    SmartDashboard.putBoolean("limit Left", left_limit.get());
    }

  public void climb() {
    climberPIDRight.setReference((-RobotContainer.getInstance().joystick.getY() * 0.5 + positionDifference) * 5700, ControlType.kVelocity);
    climberPIDLeft.setReference((-RobotContainer.getInstance().joystick.getY() * 0.5 - positionDifference) * 5700, ControlType.kVelocity);
  }

  public void stopClimber() {
    climber_motor_left.stopMotor();
    climber_motor_right.stopMotor();
  }

  public void setHeight(int encoderTicks) {
    climber_encoder_right.setPosition(encoderTicks);
    climber_encoder_left.setPosition(encoderTicks);
  }

  public void resetHeight() {
    climber_encoder_right.setPosition(0);
    climber_encoder_left.setPosition(0);
  }

  public void resetLeftHeight() {
    climber_encoder_left.setPosition(0);
  }

  public void resetRightHeight() {
    climber_encoder_right.setPosition(0);
  }

  public double getHeight() {
    return climber_encoder_right.getPosition();
  }

  public double calculatePositionDifference() {
    positionDifference = climber_motor_left.getEncoder().getPosition() - climber_motor_right.getEncoder().getPosition();
    positionDifference = Algorithms.scale(positionDifference, -100, 100, -0.1, 0.1);
    return positionDifference;
  }

  public void checkSafetyStop() {
    if(positionDifference < -Constants.CLIMBER_LEVEL_SAFETY_TICKS || positionDifference > Constants.CLIMBER_LEVEL_SAFETY_TICKS) {
      climber_motor_left.stopMotor();
      climber_motor_right.stopMotor();
    }
  }

  public void checkLimits() {
    if(right_limit.get()) {
      resetRightHeight();
    }

    if(left_limit.get()) {
      resetLeftHeight();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Height", () -> getHeight(), ticks -> setHeight((int)ticks));
    builder.addDoubleProperty("Speed", () -> climber_motor_right.get(), speed -> climber_motor_right.set(speed));
  }

}
