package frc.robot;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.exc.MismatchedInputException;
import com.kauailabs.navx.frc.AHRS;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.BallPickupMotorCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.TransportBallCommand;
import frc.robot.commands.TestCommands.StopAllBallSubsystemsCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class TestRobotContainer {
    private static TestRobotContainer mInstance;
    private final String tab = "Test Mode";

    private BallPickUpSubsystem mBallPickUpSubsystem;
    private BallShooterSubsystem mBallShooterSubsystem;
    private BallTransportSubsystem mBallTransportSubsystem;
    private ControlPanelSubsystem mControlPanelSubsystem;
    private DriveSubsystem mDriveSubsystem;
    private PneumaticSubsystem mPneumaticSubsystem;

    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private ShuffleBoardInformation controlPanelMotorSlider;
    private ShuffleBoardInformation encoderValue;
    private ShuffleBoardInformation driveForwardSlider;
    private ShuffleBoardInformation driveRotateSlider;
    private ShuffleBoardInformation loadSlider;
    private ShuffleBoardInformation shooterSlider;
    private ShuffleBoardInformation transportSlider;
    private ShuffleBoardInformation pickUpSlider;
    private ShuffleBoardInformation shooterSpeed;
    private ShuffleBoardInformation transportSensor;
    private ShuffleBoardInformation encoderValueDriveLeft;
    private ShuffleBoardInformation encoderValueDriveRight;
    private ShuffleBoardInformation pose;
    private ShuffleBoardInformation navxAngle;
    private ShuffleBoardInformation angleToTarget;
    private AimCommand aimCommand;
    private AHRS navx;
    private DriveCommand testDriveCommand;
    private BallLoaderCommand loadBallCommmand;
    private BallShooterCommand ballShooterCommand;
    private TransportBallCommand transportBallcommand;
    private BallPickupMotorCommand pickUpMotorCommand;
    private CommandBase controlPanelMotorCommand;
    private StopAllBallSubsystemsCommand stopAllBallSubsystemsCommand;
    private StopTankDriveCommand stopTankDriveCommand;

    private TestRobotContainer() {

    }

    public static TestRobotContainer getInstance() {
        if (mInstance == null) {
            mInstance = new TestRobotContainer();

            // initalize mSubsystems

            mInstance.mBallPickUpSubsystem = BallPickUpSubsystem.getInstance();
            mInstance.mBallShooterSubsystem = BallShooterSubsystem.getInstance();
            mInstance.mBallTransportSubsystem = BallTransportSubsystem.getInstance();
            mInstance.mControlPanelSubsystem = ControlPanelSubsystem.getInstance();
            mInstance.mDriveSubsystem = DriveSubsystem.getInstance();
            mInstance.mPneumaticSubsystem = PneumaticSubsystem.getInstance();

            // initialize Commands

            mInstance.testDriveCommand = new DriveCommand(() -> mInstance.getDriveForwardPos(),
                    () -> mInstance.getDriveRotatePos());
            mInstance.loadBallCommmand = new BallLoaderCommand(() -> mInstance.getLoadSlider());
            mInstance.ballShooterCommand = new BallShooterCommand(() -> mInstance.getShooterSlider(), false);
            mInstance.transportBallcommand = new TransportBallCommand(() -> mInstance.getTransportSlider(), true);
            mInstance.pickUpMotorCommand = new BallPickupMotorCommand(() -> mInstance.getPickUpSlider());
            mInstance.aimCommand = new AimCommand();
            mInstance.controlPanelMotorCommand = new CommandBase() {
                @Override
                public void execute() {
                    mInstance.mControlPanelSubsystem.setMotor(() -> mInstance.getControlPanelMotorSlider());
                }
            };
            mInstance.stopAllBallSubsystemsCommand = new StopAllBallSubsystemsCommand();
            mInstance.stopTankDriveCommand = new StopTankDriveCommand();

            TestRobotContainer.showOnShuffleBoard();

        }
        return mInstance;
        } else {
            return mInstance;
    }
    }

    private static void showOnShuffleBoard() {
        new ShuffleBoardInformation(mInstance.tab, "Extend ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.FORWARD));
        new ShuffleBoardInformation(mInstance.tab, "Reject ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.REVERSE));

        mInstance.liftReed = new ShuffleBoardInformation(mInstance.tab, "Reed switch of bottom Lift",
                mInstance.mControlPanelSubsystem.getReedLiftBotom());

        mInstance.bumperReed = new ShuffleBoardInformation(mInstance.tab, "Reed switch of front Bumper",
                mInstance.mControlPanelSubsystem.getReedBumperFront());

        mInstance.controlPanelMotorSlider = new ShuffleBoardInformation(mInstance.tab, "ControlPanelMotor", -1, 1, 0);
        mInstance.encoderValue = new ShuffleBoardInformation(mInstance.tab, "ControlPanel Motor Encoder",
                Double.valueOf(mInstance.mControlPanelSubsystem.getEncoderValue()));

        System.out.println(mInstance.mDriveSubsystem.getName());
        mInstance.driveRotateSlider = new ShuffleBoardInformation(mInstance.tab, "Rotatr Tank", -1, 1, 0);
        mInstance.driveForwardSlider = new ShuffleBoardInformation(mInstance.tab, "Drive Forward", -1, 1, 0);
        new ShuffleBoardInformation(mInstance.tab, "Stop tankdrive", mInstance.stopTankDriveCommand);
        mInstance.encoderValueDriveLeft = new ShuffleBoardInformation(mInstance.tab, "encoder drive motor left",
                mInstance.mDriveSubsystem.getEncoderLeft());
        mInstance.encoderValueDriveRight = new ShuffleBoardInformation(mInstance.tab, "encoder drive motor right",
                mInstance.mDriveSubsystem.getEncoderRight());

        mInstance.loadSlider = new ShuffleBoardInformation(mInstance.tab, "Load motor speed", -1, 1, 0);
        mInstance.transportSlider = new ShuffleBoardInformation(mInstance.tab, "Transport motor speed", -1, 1, 0);
        mInstance.shooterSlider = new ShuffleBoardInformation(mInstance.tab, "Shoot motor speed", -1, 1, 0);
        mInstance.pickUpSlider = new ShuffleBoardInformation(mInstance.tab, "Pick up motor speed", -1, 1, 0);
        new ShuffleBoardInformation(mInstance.tab, "Stop all ball subsystem motors",
                mInstance.stopAllBallSubsystemsCommand);
        mInstance.shooterSpeed = new ShuffleBoardInformation(mInstance.tab, "Shooter Speed",
                mInstance.mBallShooterSubsystem.getSpeed());
        new ShuffleBoardInformation(mInstance.tab, "Shoot ball", new ShootBallCommand(mInstance.getShooterSlider(),
                mInstance.getLoadSlider(), mInstance.getTransportSlider()));

        mInstance.pose = new ShuffleBoardInformation(mInstance.tab, "Robot Pose",
                "X: " + String.valueOf(mInstance.mDriveSubsystem.getPose().getTranslation().getX()) + "\nY: "
                        + String.valueOf(mInstance.mDriveSubsystem.getPose().getTranslation().getY())
                        + "\nRotation(degrees): " + String.valueOf(mInstance.mDriveSubsystem.getPose().getRotation()));

        mInstance.navxAngle = new ShuffleBoardInformation(mInstance.tab, "Navx angle",
                mInstance.mDriveSubsystem.getAngle());

        new ShuffleBoardInformation(mInstance.tab, "Transport ball",
                new TransportBallCommand(Constants.standardTransportSpeed, false));
        mInstance.transportSensor = new ShuffleBoardInformation(mInstance.tab, "Transport Sensor",
                mInstance.mBallTransportSubsystem.getSensor());

        mInstance.angleToTarget = new ShuffleBoardInformation(mInstance.tab, "Angle to target", (double) 0);
        new ShuffleBoardInformation(mInstance.tab, "aim", mInstance.aimCommand);
        new ShuffleBoardInformation(mInstance.tab, "PID Tunnunig", mInstance.aimCommand.getController());
        new ShuffleBoardInformation(mInstance.tab, "Drive", mInstance.testDriveCommand);
        new ShuffleBoardInformation(mInstance.tab, "set Load Ball", mInstance.loadBallCommmand);
        new ShuffleBoardInformation(mInstance.tab, "set Shoot Ball", mInstance.ballShooterCommand);
        new ShuffleBoardInformation(mInstance.tab, "set transport Ball", mInstance.transportBallcommand);
        new ShuffleBoardInformation(mInstance.tab, "set Pick up motor", mInstance.pickUpMotorCommand);
        new ShuffleBoardInformation(mInstance.tab, "set Control Panel Motor", mInstance.controlPanelMotorCommand);

    }

    public void update() {
        mInstance.liftReed.update(mInstance.mControlPanelSubsystem.getReedLiftBotom());
        mInstance.bumperReed.update(mInstance.mControlPanelSubsystem.getReedBumperFront());
        mInstance.encoderValue.update(mInstance.mControlPanelSubsystem.getEncoderValue());
        mInstance.shooterSpeed.update(mInstance.mBallShooterSubsystem.getSpeed());
        mInstance.transportSensor.update(mInstance.mBallTransportSubsystem.getSensor());
        mInstance.encoderValueDriveLeft.update(mInstance.mDriveSubsystem.getEncoderLeft());
        mInstance.encoderValueDriveRight.update(mInstance.mDriveSubsystem.getEncoderRight());
        Pose2d driveOdometry = mInstance.mDriveSubsystem.getPose();
        mInstance.pose.update(String.format("X: %f Y: %f Rot (deg): %f", driveOdometry.getTranslation().getX(),
                driveOdometry.getTranslation().getY(), driveOdometry.getRotation().getDegrees()));
        mInstance.navxAngle.update(mInstance.mDriveSubsystem.getAngle());

    }

    public DoubleSupplier getAngleToTarget() {
        return () -> mInstance.angleToTarget.getDouble();
    }

    public double getControlPanelMotorSlider() {
        return mInstance.controlPanelMotorSlider.getSliderPosition();
    }

    public double getDriveRotatePos() {
        return mInstance.driveRotateSlider.getSliderPosition();
    }

    public double getDriveForwardPos() {
        return mInstance.driveForwardSlider.getSliderPosition();
    }

    public void setDriveSlider(double forward, double rotate) {
        mInstance.driveForwardSlider.setSliderPos(forward);
        mInstance.driveRotateSlider.setSliderPos(rotate);
    }

    public double getLoadSlider() {
        return mInstance.loadSlider.getSliderPosition();
    }

    public double getTransportSlider() {
        return mInstance.transportSlider.getSliderPosition();
    }

    public double getShooterSlider() {
        return mInstance.shooterSlider.getSliderPosition();
    }

    public double getPickUpSlider() {
        return mInstance.pickUpSlider.getSliderPosition();
    }

    public void setShooterSliderPos(double pos) {
        mInstance.shooterSlider.setSliderPos(pos);
    }

    public void setLoadSliderPos(double pos) {
        mInstance.loadSlider.setSliderPos(pos);
    }

    public void setTransportSliderPos(double pos) {
        mInstance.transportSlider.setSliderPos(pos);
    }

    public void setPickUpSliderPos(double pos) {
        mInstance.pickUpSlider.setSliderPos(pos);
    }

    public double getAngle() {
        return mDriveSubsystem.getAngle();
    }
    // public DoubleSupplier getAngleToTarget() {
    // return () -> 0;
    // }

    // public double getControlPanelMotorSlider() {
    // return 0;
    // }

    // public double getDriveRotatePos() {
    // return 0;
    // }

    // public double getDriveForwardPos() {
    // return 0;
    // }

    // public void setDriveSlider(double forward, double rotate) {

    // }

    // public double getLoadSlider() {
    // return 0;
    // }

    // public double getTransportSlider() {
    // return 0;
    // }

    // public double getShooterSlider() {
    // return 0;
    // }

    // public double getPickUpSlider() {
    // return 0;
    // }

    // public void setShooterSliderPos(double pos) {

    // }

    // public void setLoadSliderPos(double pos) {

    // }

    // public void setTransportSliderPos(double pos) {

    // }

    // public void setPickUpSliderPos(double pos) {

    // }
}