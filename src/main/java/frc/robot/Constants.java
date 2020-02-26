/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANDigitalInput;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.subsystems.ControlPanelSubsystem.ColorDetected;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    /** 
     * Joystick button constants used in {@link RobotContainer}
     */ 
    public static final int JOYSTICK_PORT = 0;
    public static final int STEERING_WHEEL_PORT = 1;
    public static final int DEACTIVATE_SECUTITY_MECHANISMS_BUTTON_NUMBER = 7;

    // No auto mechanisms buttons
    public static final int LOADER_NO_AUTOMECHANISMS_BUTTON_NUMBER = 11;
    public static final int CONTROL_PANEL_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMPER = 12;
    public static final int TRANSPORT_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER = 10;
    public static final int SHOOTER_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER = 9;

    // BallPickUp Buttons
    public static final int BALL_PICKUP_MOTOR_BUTTON_NUMPER = 2;
    public static final int BALL_PICKUP_EXTEND_AND_RETRACT_PROTECTOR = 8;
    
    // ControlPanel Buttons
    public static final int EXTEND_AND_RETRACT_CONTROL_PANEL_MODULE_BUTTON_NUMBER = 5;
    public static final int GO_TO_COLOR_BUTTON_NUMBER = 3;

    // Shoot and transport Buttons
    public static final int SHOOT_BUTTON_NUMBER = 1;
    public static final int AIM_BUTTON_NUMBER = 4;

    // Climbing Buttons
    public static final int ACTIVATE_CLIMBING_BUTTON_NUMBER = 6;
    public static final int CLIMB_TO_BOTTOM_BUTTON_NUMBER = 4;

    /**
     * Activate subsystems constants
    */
    public static final boolean CONTROL_PANEL_SUBSYSTEM_ENABLED = true;
    public static boolean DRIVE_SUBSYSTEM_ENABLED = true;
    public static final boolean PNEUMATIC_SUBSYSTEM_ENABLED = false;
    public static final boolean BALL_SHOOTER_SUBSYSTEM_ENABLED = true;
    public static final boolean BALL_TRANSPORT_SUBSYSTEM_ENABLED = true;
    public static final boolean BALL_PICKUP_SUBSYSTEM_ENABLED = true;
    public static boolean CLIMBER_SUBSYSTEM_ENABLED = false;

    /** 
   * Pneumatic Subsystem constants used in {@link PneumaticSubsystem}
   */

   public static final int PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID = 30;
   public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID = 2;
   public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID = 3;
   public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_EXTEND_ID = 0;
   public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_RETRACT_ID = 1;


  /**
   * Control Panel Subsystem constants used in {@link ControlPanelSubsystem}.
   */

  public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_BOTTOM_REED_DI_ID = 0;
  public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_TOP_REED_DI_ID = 1;
  public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_BACK_REED_DI_ID = 2;
  public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_FRONT_REED_DI_ID = 3;
  public static final I2C.Port CONTROL_PANEL_SUBSYSTEM_COLOR_SENSOR_I2C_PORT = I2C.Port.kOnboard;
  public static final int CONTROL_PANEL_SUBSYSTEM_MOTOR_CAN_ID = 16;
  public static final ArrayList<ColorDetected> colors = new ArrayList<ColorDetected>(Arrays.asList(ColorDetected.RED, ColorDetected.YELLOW, ColorDetected.BLUE, ColorDetected.GREEN));
  public static final double CONTROL_PANEL_MOTOR_SPEED = 0.2;

  /**
   * Ball Pickup Subsystem constants used in {@link BallPickupSubsystem}
   */

  public static final int BALL_PICKUP_SUBSYSTEM_EXTEND_ID = 6;
  public static final int BALL_PICKUP_SUBSYSTEM_RETRACT_ID = 7;
  public static final int BALL_PICKUP_PROTECTER_EXTEND_ID = 5;
  public static final int BALL_PICKUP_PROTECTER_RETRACT_ID = 4;

  /** 
 * Drive Subsystem constants used in {@link DriveSubsystem}.
 */

 public static final double AIM_MAX_SPEED = 0.2;
 public static final int Tank_Left_BACK_ID = 11;
 public static final int Tank_Left_FRONT_ID = 10;
 public static final int Tank_Right_BACK_ID = 13;
 public static final int Tank_Right_FRONT_ID = 12; 
 public static final double WHEEL_CIRCUMFERENCE = 0.4787787204060999;  // IN METER
 public static final double GEARBOX_TRANSLATION = 10.71;
 public static final double PID_TOLERANZ = 5; // Angle in degrees

 // Standart speed constants used in all ball Subsystems

 public static final DoubleSupplier standardShooterSpeed = () -> 1;
 public static final DoubleSupplier standardLoaderSpeed = () -> 0.25;
 public static final DoubleSupplier standardTransportSpeed = () -> 0.25;
 public static final DoubleSupplier standardPickUpMotorSpeed = () -> 0.5;

  // Ball subsytems can IDs
  public static final int BALL_SHOOTER_SUBSYSTEM_LOADER_CAN_ID = 21;
  public static final int BALL_SHOOTER_SUBSYSTEM_SHOOTER_LEFT_CAN_ID = 22;
  public static final int BALL_SHOOTER_SUBSYSTEM_SHOOTER_RIGHT_CAN_ID = 23;
  public static final int BALL_TRANSPORT_MOTOR_CAN_ID = 15;  
  public static final int BALL_PICKUP_MOTOR_CAN_ID = 14;

  public static final int BALL_TRANSPORT_SENSOR_DIO_PORT = 0;

  /**
   * Climber Subsystem constants used in {@link ClimberSusystem}
   */

  public static final double kPclimber = 0.0002; 
  public static final double kIclimber = 0; 
  public static final double kDclimber = 0.0001; 
  public static final double kIzclimber = 0; 
  public static final double kFFclimber = 0.000; 
  public static final double kMaxOutputclimber = 0.7; 
  public static final double kMinOutputclimber = -0.7; 
  public static final double maxRPMclimber = 5700;

  public static final int MOTOR_UPPER_THROWER_SHAFT_RIGHT_ID = 22;

  public static final int MOTOR_CLIMBER_LEFT_ID = 24;
  public static final int MOTOR_CLIMBER_RIGHT_ID = 25;

  public static final double CLIMBER_LEVEL_SAFETY_TICKS = 10;

  public static final double standardClimberSpeed = 0.5;

  public static final double CLIMBER_TOP_POSITION = 95.5; // In encoder ticks
}