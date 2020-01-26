/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static DriveSubsystem mInstance;
  private WPI_TalonSRX tankLeftFront = new WPI_TalonSRX(Constants.Tank_Left_1_ID);
  private WPI_TalonSRX tankLeftBack = new WPI_TalonSRX(Constants.Tank_Left_2_ID);
  private WPI_TalonSRX tankRightFront = new WPI_TalonSRX(Constants.Tank_Right_1_ID);
  private WPI_TalonSRX tankRightBack = new WPI_TalonSRX(Constants.Tank_Right_2_ID);
  
  
  private DriveSubsystem() {
    tankLeftFront.configFactoryDefault();
    tankLeftBack.configFactoryDefault();
    tankRightFront.configFactoryDefault();
    tankRightBack.configFactoryDefault();
  }

  public static DriveSubsystem getInstace() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
      return mInstance;
    } else {
      return mInstance;
    }
  }

  @Override
  public void periodic() {
  }
  private void driveLeft(double l_percentage){
      tankLeftFront.set(l_percentage);
      tankLeftBack.set(l_percentage);
  }
  private void driveRight(double r_percentage){
      tankRightBack.set(r_percentage);
      tankRightFront.set(r_percentage);
  }
  public void drive(double l_percentage, double r_percentage){
    driveRight(r_percentage);
    driveLeft(l_percentage);
  }

  public void stopDrive() {
    drive(0, 0);
  }
}

