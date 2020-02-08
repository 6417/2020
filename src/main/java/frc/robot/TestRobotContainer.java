package frc.robot;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.commands.AimCommand;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.TransportBallCommand;
import frc.robot.commands.TestCommands.StopAllBallSubsystemsCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;


public class TestRobotContainer {
    private static TestRobotContainer mInstance;
    private final String tab = "Test Mode";
    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private ShuffleBoardInformation controlPanelMotorSlider;
    private ShuffleBoardInformation encoderValue;
    private static ShuffleBoardInformation driveLeftSlider;
    private static ShuffleBoardInformation driveRightSlider;
    private ShuffleBoardInformation loadSlider;
    private ShuffleBoardInformation shooterSlider;
    private ShuffleBoardInformation transportSlider;
    private ShuffleBoardInformation  pickUpSlider;
    private ShuffleBoardInformation shooterSpeed;
    private ShuffleBoardInformation transportSensor;
    private ShuffleBoardInformation encoderValueDriveLeft;
    private ShuffleBoardInformation encoderValueDriveRight;
    private ShuffleBoardInformation pose;
    private ShuffleBoardInformation navxAngle;
    private ShuffleBoardInformation angleToTarget;
    private AimCommand aimCommand;
    private AHRS navx;

    private TestRobotContainer() {
        showOnShuffleBoard();

        try {
            navx = new AHRS(SPI.Port.kMXP);
            // ahrs = new AHRS(SerialPort.Port.kUSB1);
            navx.enableLogging(true);
          } catch (RuntimeException ex) {
           DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
          }
    }

    public double getAngle() {
        return navx.getAngle();
    }

    public static TestRobotContainer getInstance() {
        if (mInstance == null) {
            mInstance = new TestRobotContainer();
            return mInstance;
        } else {
            return mInstance;
        }
    }

    private void showOnShuffleBoard() {
        new ShuffleBoardInformation(tab, "Extend ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.FORWARD));
        new ShuffleBoardInformation(tab, "Reject ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.REVERSE));

        liftReed = new ShuffleBoardInformation(tab, "Reed switch of bottom Lift",
                ControlPanelSubsystem.getInstance().getReedLiftBotom());

        bumperReed = new ShuffleBoardInformation(tab, "Reed switch of front Bumper",
                ControlPanelSubsystem.getInstance().getReedBumperFront());

        controlPanelMotorSlider = new ShuffleBoardInformation(tab, "ControlPanelMotor", -1, 1, 0);
        encoderValue = new ShuffleBoardInformation(tab, "ControlPanel Motor Encoder",
                Double.valueOf(ControlPanelSubsystem.getInstance().getEncoderValue()));
        

        driveRightSlider = new ShuffleBoardInformation(tab, "Turn right", -1, 1, 0);
        driveLeftSlider = new ShuffleBoardInformation(tab, "Turn left", -1, 1, 0);
        new ShuffleBoardInformation(tab, "Stop tankdrive", new StopTankDriveCommand());
        encoderValueDriveLeft = new ShuffleBoardInformation(tab, "encoder drive motor left", DriveSubsystem.getInstance().getEncoderLeft());
        encoderValueDriveRight = new ShuffleBoardInformation(tab, "encoder drive motor right", DriveSubsystem.getInstance().getEncoderRight());

        loadSlider = new ShuffleBoardInformation(tab, "Load motor speed", -1, 1, 0);
        transportSlider = new ShuffleBoardInformation(tab, "Transport motor speed", -1, 1, 0);
        shooterSlider = new ShuffleBoardInformation(tab, "Shoot motor speed", -1, 1, 0);
        pickUpSlider = new ShuffleBoardInformation(tab, "Pick up motor speed", -1,  1, 0);
        new ShuffleBoardInformation(tab, "Stop all ball subsystem motors", new StopAllBallSubsystemsCommand());
        shooterSpeed = new ShuffleBoardInformation(tab, "Shooter Speed", BallShooterSubsystem.getInstance().getSpeed());
        new ShuffleBoardInformation(tab, "Shoot ball", new ShootBallCommand(getShooterSlider(), getLoadSlider(), getTransportSlider()));

        pose = new ShuffleBoardInformation(tab, "Robot Pose", "X: " + String.valueOf(DriveSubsystem.getInstance().getPose().getTranslation().getX()) + 
            "\nY: " + String.valueOf(DriveSubsystem.getInstance().getPose().getTranslation().getY()) + 
            "\nRotation(degrees): " + String.valueOf(DriveSubsystem.getInstance().getPose().getRotation()));

        navxAngle = new ShuffleBoardInformation(tab, "Navx angle", DriveSubsystem.getInstance().getAngle());

        new ShuffleBoardInformation(tab, "Transport ball", new TransportBallCommand(Constants.standardTransportSpeed, false));
        transportSensor = new ShuffleBoardInformation(tab, "Transport Sensor", BallTransportSubsystem.getInstance().getSensor());

        angleToTarget = new ShuffleBoardInformation(tab, "Angle to target", (double)0);
        System.out.println("ShowOnShuffleBoaed running");
        aimCommand = new AimCommand();
        new ShuffleBoardInformation(tab, "aim", aimCommand);
        new ShuffleBoardInformation(tab, "PID Tunnunig", aimCommand.getController());
        System.out.println("showOnShuffleBoard finished");
    }

    public void update() {
        liftReed.update(ControlPanelSubsystem.getInstance().getReedLiftBotom());            
        bumperReed.update(ControlPanelSubsystem.getInstance().getReedBumperFront());
        encoderValue.update(ControlPanelSubsystem.getInstance().getEncoderValue());
        shooterSpeed.update(BallShooterSubsystem.getInstance().getSpeed());
        transportSensor.update(BallTransportSubsystem.getInstance().getSensor());
        encoderValueDriveLeft.update(DriveSubsystem.getInstance().getEncoderLeft());
        encoderValueDriveRight.update(DriveSubsystem.getInstance().getEncoderRight());
        Pose2d driveOdometry = DriveSubsystem.getInstance().getPose();
        pose.update(String.format("X: %f Y: %f Rot (deg): %f",
            driveOdometry.getTranslation().getX(),
            driveOdometry.getTranslation().getY(),
            driveOdometry.getRotation().getDegrees()
            ));
        navxAngle.update(DriveSubsystem.getInstance().getAngle());
    }

    public DoubleSupplier getAngleToTarget() {
        return () -> angleToTarget.getDouble();
    }

    public double getControlPanelMotorSlider() {
        return controlPanelMotorSlider.getSliderPosition();
    }

    public double getDriveRightPos() {
        return driveRightSlider.getSliderPosition();
    }

    public double getDriveLeftPos() {
        return driveLeftSlider.getSliderPosition();
    }

    public static void setDriveSlider(double left, double right) {
        driveLeftSlider.setSliderPos(left);
        driveRightSlider.setSliderPos(right);
    }

    public double getLoadSlider() {
        return loadSlider.getSliderPosition();
    }

    public double getTransportSlider() {
        return transportSlider.getSliderPosition();
    }

    public double getShooterSlider() {
        return shooterSlider.getSliderPosition();
    }

    public double getPickUpSlider() {
        return pickUpSlider.getSliderPosition();
    }

    public void setShooterSliderPos(double pos) {
        shooterSlider.setSliderPos(pos);
    }

    public void setLoadSliderPos(double pos) {
        loadSlider.setSliderPos(pos);
    }

    public void setTransportSliderPos(double pos) {
        transportSlider.setSliderPos(pos);
    }

    public void setPickUpSliderPos(double pos) {
        pickUpSlider.setSliderPos(pos);
    }
}