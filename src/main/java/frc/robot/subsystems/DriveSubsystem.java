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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax tankLeftFront;
  private CANSparkMax tankLeftBack;
  private CANSparkMax tankRightFront;
  private CANSparkMax tankRightBack;
  private DifferentialDrive diffdrive;
  
  
  protected DriveSubsystem() {
    constructor();
  }

  protected void constructor() {
    tankLeftFront = new CANSparkMax(Constants.Tank_Left_FRONT_ID, MotorType.kBrushless);
    tankLeftBack = new CANSparkMax(Constants.Tank_Left_BACK_ID, MotorType.kBrushless);
    tankRightFront = new CANSparkMax(Constants.Tank_Right_FRONT_ID, MotorType.kBrushless);
    tankRightBack = new CANSparkMax(Constants.Tank_Right_BACK_ID, MotorType.kBrushless);
    diffdrive = new DifferentialDrive(tankLeftBack, tankRightBack);

    tankRightBack.restoreFactoryDefaults();
    
    tankRightFront.restoreFactoryDefaults();
    tankRightFront.follow(tankRightBack, true);
    
    tankLeftBack.restoreFactoryDefaults();

    tankLeftFront.restoreFactoryDefaults();
    tankLeftFront.follow(tankLeftBack, true);
    
    diffdrive.setRightSideInverted(false);
  }

  public static DriveSubsystem getInstance() {
    if (!Constants.DRIVE_SUBSYSTEM_ENABLED && mInstance == null) {
      mInstance = new EmptyDriveSubsystem();
    } else if (mInstance == null) {
      mInstance = new DriveSubsystem();
      mInstance.setDefaultCommand(new DriveCommand());
    } 
    return mInstance;
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
    diffdrive.arcadeDrive(-RobotContainer.joystick.getY(), RobotContainer.joystick.getX());
  }

  public void drive(double xSpeed, double ySpeed) {
    diffdrive.arcadeDrive(xSpeed, ySpeed);
  }

  public void stopDrive() {
    getDefaultCommand().cancel();
  }

  public void resetEncoders(){
    tankLeftBack.getEncoder().setPosition(0);
    tankRightBack.getEncoder().setPosition(0);
  }

  public double getEncoderLeft() {
    return tankLeftBack.getEncoder().getPosition();
  }

  public double getEncoderRight() {
    return tankRightBack.getEncoder().getPosition();
  }
}

