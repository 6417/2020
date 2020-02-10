package frc.robot.subsystems.emptySubsystems;

import frc.robot.subsystems.DriveSubsystem;

public class EmptyDriveSubsystem extends DriveSubsystem {
    public EmptyDriveSubsystem() {
        super();
    } 

    protected void constructor() {

    }

    public void driveLeft(double l_percentage) {

    }

    public void driveRight(double r_percentage) {

    }

    public void drive(double xSpeed, double ySpeed) {

    }

    public void drive() {

    }

    public void stopDrive() {

    }

    public void resetEncoders(){

    }

    public double getEncoderLeft(){
        return 0;
    }
    
    public double getEncoderRight(){
        return 0;
    }

    public double getEncoderLeftMetric() {
        return 0;
    }

    public double getEncoderRightMetric(){
        return 0;
    }

    public double getAngle(){
        return 0;
    }


    public Pose2d getPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    @Override
    public void periodic() {
        
    }
}