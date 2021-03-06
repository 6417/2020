/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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
   * Pneumatic Subsystem constants used in {@link PneumaticSubsystem}
   */

   public static final int PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID = 20;
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
}
