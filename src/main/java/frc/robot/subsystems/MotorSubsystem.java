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

public class MotorSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public WPI_TalonSRX tankLeftFront = new WPI_TalonSRX(Constants.Tank_Left_1_ID);
  public WPI_TalonSRX tankLeftBack = new WPI_TalonSRX(Constants.Tank_Left_2_ID);
  public WPI_TalonSRX tankRightFront = new WPI_TalonSRX(Constants.Tank_Right_1_ID);
  public WPI_TalonSRX tankRightBack = new WPI_TalonSRX(Constants.Tank_Right_2_ID);
  
  
  public MotorSubsystem() {
    tankLeftFront.configFactoryDefault();
    tankLeftBack.configFactoryDefault();
    tankRightFront.configFactoryDefault();
    tankRightBack.configFactoryDefault();
  }

  @Override
  public void periodic() {
  }
  public void driveLeft(double l_percentage){
      tankLeftFront.set(l_percentage);
      tankLeftBack.set(l_percentage);
  }
  public void driveRight(double r_percentage){
      tankRightBack.set(r_percentage);
      tankRightFront.set(r_percentage);
  }
  public void drive(double l_percentage, double r_percentage){
    driveRight(r_percentage);
    driveLeft(l_percentage);
  }
}

