package frc.robot.subsystems.emptySubsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;

public class EmptyDriveSubsystem extends DriveSubsystem {
    public EmptyDriveSubsystem() {
        super();
    } 

    @Override
    protected void constructor() {

    }

    @Override
    public void driveLeft(double l_percentage) {

    }

    @Override
    public void driveRight(double r_percentage) {

    }

    @Override
    public void drive(double xSpeed, double ySpeed) {

    }
    @Override
    public void drive() {

    }

    @Override
    public void stopDrive() {

    }

    @Override
    public void resetEncoders(){

    }

    @Override
    public double getEncoderLeft(){
        return 0;
    }
    
    @Override
    public double getEncoderRight(){
        return 0;
    }

    @Override
    public double getEncoderLeftMetric() {
        return 0;
    }

    @Override
    public double getEncoderRightMetric(){
        return 0;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    @Override
    public void periodic() {
    }
}