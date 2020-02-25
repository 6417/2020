/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
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
  public static DriveSubsystem mInstance;
  private static CANSparkMax tankLeftFront;
  private static CANSparkMax tankLeftBack;
  private static CANSparkMax tankRightFront;
  private static CANSparkMax tankRightBack;
  private static DifferentialDrive diffdrive;
  private static AHRS navx;
  private static DifferentialDriveOdometry m_odometry;

  private static LatchedBoolean firsGetInstance = new LatchedBoolean(EdgeDetection.RISING);

  protected DriveSubsystem() {
    constructor();
  }

  protected void constructor() {
    if (firsGetInstance.update(true)) {
      navx = RobotContainer.navx;
      tankLeftFront = new CANSparkMax(Constants.Tank_Left_FRONT_ID, MotorType.kBrushless);
      tankLeftBack = new CANSparkMax(Constants.Tank_Left_BACK_ID, MotorType.kBrushless);
      tankRightFront = new CANSparkMax(Constants.Tank_Right_FRONT_ID, MotorType.kBrushless);
      tankRightBack = new CANSparkMax(Constants.Tank_Right_BACK_ID, MotorType.kBrushless);
      diffdrive = new DifferentialDrive(tankLeftBack, tankRightBack);
      
      navx.reset();
      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(new Rotation2d(navx.getAngle()), new Pose2d(0, 0, new Rotation2d(0)));
    
      
      //Configure motors

      tankRightBack.restoreFactoryDefaults();
      tankRightBack.getEncoder().setPositionConversionFactor(-1);
      
      tankRightFront.restoreFactoryDefaults();
      tankRightFront.follow(tankRightBack);
      
      tankLeftBack.restoreFactoryDefaults();
      tankLeftBack.getEncoder().setPositionConversionFactor(1);

      tankLeftFront.restoreFactoryDefaults();
      tankLeftFront.follow(tankLeftBack);
      
      diffdrive.setRightSideInverted(true);
    }
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
    m_odometry.update(Rotation2d.fromDegrees(-navx.getAngle()), getEncoderLeftMetric(), getEncoderRightMetric());
  }

  public void driveLeft(double l_percentage){
      tankLeftBack.set(l_percentage);
  }

  public void driveRight(double r_percentage){    
      tankRightBack.set(r_percentage);
  }

  public void drive(){
    diffdrive.arcadeDrive(-RobotContainer.joystick.getY(), RobotContainer.steeringWheel.getX(), true);
  }

  public void drive(double xSpeed, double ySpeed) {
    diffdrive.arcadeDrive(xSpeed, ySpeed);
  }

  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    drive(xSpeed.getAsDouble(), ySpeed.getAsDouble());
  }

  public void stopDrive() {
    getDefaultCommand().cancel();
  }

  public void resetEncoders(){
    tankLeftBack.getEncoder().setPosition(0);
    tankRightBack.getEncoder().setPosition(0);
  }

  public void resetNavx(){
    navx.reset();
  }

  public double getEncoderLeft() {
    return tankLeftBack.getEncoder().getPosition();
  }

  public double getEncoderRight() {
    return -tankRightBack.getEncoder().getPosition();
  }

  public double getEncoderLeftMetric() {
    return (tankLeftBack.getEncoder().getPosition()/Constants.GEARBOX_TRANSLATION) * Constants.WHEEL_CIRCUMFERENCE;
  }

  public double getEncoderRightMetric() {
    return -(tankRightBack.getEncoder().getPosition()/Constants.GEARBOX_TRANSLATION) * Constants.WHEEL_CIRCUMFERENCE;
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }
 
  public double getAngle() {
    return navx.getAngle();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}