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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
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
  private AHRS navx;
  private DifferentialDriveOdometry m_odometry;
  
  
  protected DriveSubsystem() {
    constructor();
  }

  protected void constructor() {
    tankLeftFront = new WPI_TalonSRX(Constants.Tank_Left_FRONT_ID);
    tankLeftBack = new WPI_TalonSRX(Constants.Tank_Left_BACK_ID);
    tankRightFront = new WPI_TalonSRX(Constants.Tank_Right_FRONT_ID);
    tankRightBack = new WPI_TalonSRX(Constants.Tank_Right_BACK_ID);
    diffdrive = new DifferentialDrive(tankLeftBack, tankRightBack);
    
    try {
      navx = new AHRS(SPI.Port.kMXP);
      // ahrs = new AHRS(SerialPort.Port.kUSB1);
      navx.enableLogging(true);
    } catch (RuntimeException ex) {
     DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    
    navx.reset();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(navx.getAngle()), new Pose2d(0, 0, new Rotation2d(0)));
  
    
     

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

  public double getEncoderLeftMetric() {
    return Constants.WHEEL_SCOPE * (tankLeftBack.getSelectedSensorPosition() / Constants.TICKS_PER_ROTATION);
  }

  public double getEncoderRightMetric() {
    return -Constants.WHEEL_SCOPE * (tankRightBack.getSelectedSensorPosition() / Constants.TICKS_PER_ROTATION);
  }

  

  public Pose2d getPose(){
    return m_odometry.update(new Rotation2d(navx.getAngle()), getEncoderLeftMetric(), getEncoderRightMetric());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}

