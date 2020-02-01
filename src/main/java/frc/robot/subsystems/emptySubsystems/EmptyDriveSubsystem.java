package frc.robot.subsystems.emptySubsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

    public Pose2d getPose() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    public double getEncoderLeftMetric() {
        return 0;
    }

    public double getEncoderRightMetric(){
        return 0;
    }

    @Override
    public void periodic() {
        
    }
}