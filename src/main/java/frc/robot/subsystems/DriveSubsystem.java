/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.emptySubsystems.EmptyDriveSubsystem;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static DriveSubsystem mInstance;
  private WPI_TalonSRX tankLeftFront;
  private WPI_TalonSRX tankLeftBack;
  private WPI_TalonSRX tankRightFront;
  private WPI_TalonSRX tankRightBack;
  private DifferentialDrive diffdrive;
  
  
  protected DriveSubsystem() {
    constructor();
  }

  protected void constructor() {
    tankLeftFront = new WPI_TalonSRX(Constants.Tank_Left_FRONT_ID);
    tankLeftBack = new WPI_TalonSRX(Constants.Tank_Left_BACK_ID);
    tankRightFront = new WPI_TalonSRX(Constants.Tank_Right_FRONT_ID);
    tankRightBack = new WPI_TalonSRX(Constants.Tank_Right_BACK_ID);
    diffdrive = new DifferentialDrive(tankLeftBack, tankRightBack);

    tankRightBack.configFactoryDefault();
    tankRightBack.setInverted(InvertType.InvertMotorOutput);
    
    tankRightFront.configFactoryDefault();
    tankRightFront.follow(tankRightBack);
    tankRightFront.setInverted(InvertType.FollowMaster);
    
    tankLeftBack.configFactoryDefault();
    tankLeftBack.setInverted(InvertType.None);

    tankLeftFront.configFactoryDefault();
    tankLeftFront.follow(tankLeftBack);
    tankLeftFront.setInverted(InvertType.FollowMaster);

    tankLeftBack.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    tankRightBack.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    diffdrive.setRightSideInverted(false);
  }

  public static DriveSubsystem getInstance() {
    if (Constants.DRIVE_SUBSYSTEM_ENABLED) {
      if (mInstance == null) {
        mInstance = new DriveSubsystem();
        mInstance.setDefaultCommand(new DriveCommand());
        return mInstance;
      } else {
        return mInstance;
      }
    } else {
      return new EmptyDriveSubsystem();
    }
  }

  @Override
  public void periodic() {
  }

  public void driveLeft(double l_percentage){
      tankLeftBack.set(l_percentage);
  }

  public void driveRight(double r_percentage){    
      tankRightBack.set(r_percentage);
  }

  public void drive(){
    diffdrive.arcadeDrive(-RobotContainer.mainDriver.getY(), RobotContainer.mainDriver.getX());
  }

  public void drive(double xSpeed, double ySpeed) {
    diffdrive.arcadeDrive(xSpeed, ySpeed);
  }

  public void stopDrive() {
    getDefaultCommand().cancel();
  }

  public void resetEncoders(){
    tankLeftBack.setSelectedSensorPosition(0);
    tankRightBack.setSelectedSensorPosition(0);
  }

  public double getEncoderLeft() {
    return tankLeftBack.getSelectedSensorPosition();
  }

  public double getEncoderRight() {
    return tankRightBack.getSelectedSensorPosition();
  }
}

