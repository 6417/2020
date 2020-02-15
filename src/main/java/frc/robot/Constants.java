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
    // Joystick button constants used in {@link RobotContainer}
    public static final int JOYSTICK_PORT = 0;
    public static final int DEACTIVATE_SECUTITY_MECHANISMS_BUTTON_NUMBER = 7;

    public static final int LOADER_NO_AUTOMECHANISMS_BUTTON_NUMBER = 11;
    public static final int CONTROL_PANEL_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMPER = 12;
    public static final int TRANSPORT_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER = 10;
    public static final int SHOOTER_MOTOR_NO_AUTOMECHANISMS_BUTTON_NUMBER = 9;


    // Activate subsystems constants
    public static final boolean CONTROL_PANEL_SUBSYSTEM_ENABLED = false;
    public static final boolean DRIVE_SUBSYSTEM_ENABLED = false;
    public static final boolean PNEUMATIC_SUBSYSTEM_ENABLED = false;
    public static final boolean BALL_SHOOTER_SUBSYSTEM_ENABLED = false;
    public static final boolean BALL_TRANSPORT_SUBSYSTEM_ENABLED = false;
    public static final boolean BALL_PICKUP_SUBSYSTEM_ENABLED = false;

    /**
   * Pneumatic Subsystem constants used in {@link PneumaticSubsystem}
   */

   public static final int PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID = 30;
   public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID = 0;
   public static final int CONTROL_PANEL_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID = 1;
   public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_EXTEND_ID = 2;
   public static final int CONTROL_PANEL_SUBSYSTEM_BUMPER_SOLENOID_RETRACT_ID = 3;


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
  public static final double CONTROL_PANEL_MOTOR_SPEED = 0.45;

  /**
   * Ball Pickup Subsystem constants used in {@link BallPickupSubsystem}
   */

   public static final int BALL_PICKUP_SUBSYSTEM_EXTEND_ID = 4;
   public static final int BALL_PICKUP_SUBSYSTEM_RETRACT_ID = 5;

  /** 
 * Motor Subsystem constants used in {@link MotorSubsystem}.
 */

 public static final double AIM_MAX_SPEED = 0.2;
 public static final int Tank_Left_BACK_ID = 11;
 public static final int Tank_Left_FRONT_ID = 10;
 public static final int Tank_Right_BACK_ID = 13;
 public static final int Tank_Right_FRONT_ID = 12; 
 public static final double WHEEL_CIRCUMFERENCE = 0.4787787204060999;  // IN METER
 public static final double PID_TOLERANZ = 5; // Angle in degrees

 // Standart speed constants used in all ball Subsystems

 public static final DoubleSupplier standardShooterSpeed = () -> 0.9;
 public static final DoubleSupplier standardLoaderSpeed = () -> 0.25;
 public static final DoubleSupplier standardTransportSpeed = () -> 0.25;

  // Ball subsytems can IDs
  public static final int BALL_SHOOTER_SUBSYSTEM_LOADER_CAN_ID = 21;
  public static final int BALL_SHOOTER_SUBSYSTEM_SHOOTER_LEFT_CAN_ID = 22;
  public static final int BALL_SHOOTER_SUBSYSTEM_SHOOTER_RIGHT_CAN_ID = 23;
  public static final int BALL_TRANSPORT_MOTOR_CAN_ID = 15;  
  public static final int BALL_PICKUP_MOTOR_CAN_ID = 14;

  public static final int BALL_TRANSPORT_SENSOR_DIO_PORT = 0;
}