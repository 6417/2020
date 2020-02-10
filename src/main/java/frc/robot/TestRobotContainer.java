package frc.robot;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.BallPickupMotorCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PneumaticLiftCommand;
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

    private BallPickUpSubsystem mBallPickUpSubsystem = BallPickUpSubsystem.getInstance();
    private BallShooterSubsystem mBallShooterSubsystem = BallShooterSubsystem.getInstance();
    private BallTransportSubsystem mBallTransportSubsystem = BallTransportSubsystem.getInstance();
    private static ControlPanelSubsystem mControlPanelSubsystem = ControlPanelSubsystem.getInstance();
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private PneumaticSubsystem mPneumaticSubsystem = PneumaticSubsystem.getInstance();

    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private static ShuffleBoardInformation controlPanelMotorSlider;
    private ShuffleBoardInformation encoderValue;
    private static ShuffleBoardInformation driveForwardSlider;
    private static ShuffleBoardInformation driveRotateSlider;
    private static ShuffleBoardInformation loadSlider;
    private static ShuffleBoardInformation shooterSlider;
    private static ShuffleBoardInformation transportSlider;
    private static ShuffleBoardInformation pickUpSlider;
    private ShuffleBoardInformation shooterSpeed;
    private ShuffleBoardInformation transportSensor;
    private ShuffleBoardInformation encoderValueDriveLeft;
    private ShuffleBoardInformation encoderValueDriveRight;
    private ShuffleBoardInformation pose;
    private ShuffleBoardInformation navxAngle;
    private ShuffleBoardInformation angleToTarget;
    private static AimCommand aimCommand;
    private AHRS navx;
    private static DriveCommand testDriveCommand;
    private static BallLoaderCommand loadBallCommmand;
    private static BallShooterCommand ballShooterCommand;
    private static TransportBallCommand transportBallcommand;
    private static BallPickupMotorCommand pickUpMotorCommand;
    private static CommandBase controlPanelMotorCommand;
    private static StopAllBallSubsystemsCommand stopAllBallSubsystemsCommand;
    private static StopTankDriveCommand stopTankDriveCommand;

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

    private static void initializeCommands() {
        testDriveCommand = new DriveCommand(() -> getDriveForwardPos(), () -> getDriveRotatePos());
        loadBallCommmand = new BallLoaderCommand(() -> getLoadSlider());
        ballShooterCommand = new BallShooterCommand(() -> getShooterSlider(), false);
        transportBallcommand = new TransportBallCommand(() -> getTransportSlider(), true);
        pickUpMotorCommand = new BallPickupMotorCommand(() -> getPickUpSlider());
        aimCommand = new AimCommand();
        controlPanelMotorCommand = new CommandBase() {
            @Override
            public void execute() {
                mControlPanelSubsystem.setMotor(getControlPanelMotorSlider());
            }
        };
        stopAllBallSubsystemsCommand = new StopAllBallSubsystemsCommand();
        stopTankDriveCommand = new StopTankDriveCommand();
    }

    public double getAngle() {
        return navx.getAngle();
    }

    public static TestRobotContainer getInstance() {
        if (mInstance == null) {
            mInstance = new TestRobotContainer();
            initializeCommands();
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
                mControlPanelSubsystem.getReedLiftBotom());

        bumperReed = new ShuffleBoardInformation(tab, "Reed switch of front Bumper",
                mControlPanelSubsystem.getReedBumperFront());

        controlPanelMotorSlider = new ShuffleBoardInformation(tab, "ControlPanelMotor", -1, 1, 0);
        encoderValue = new ShuffleBoardInformation(tab, "ControlPanel Motor Encoder",
                Double.valueOf(mControlPanelSubsystem.getEncoderValue()));


        System.out.println(mDriveSubsystem.getName());
        driveRotateSlider = new ShuffleBoardInformation(tab, "Rotatr Tank", -1, 1, 0);
        driveForwardSlider = new ShuffleBoardInformation(tab, "Drive Forward", -1, 1, 0);
        new ShuffleBoardInformation(tab, "Stop tankdrive", stopTankDriveCommand);
        encoderValueDriveLeft = new ShuffleBoardInformation(tab, "encoder drive motor left",
                mDriveSubsystem.getEncoderLeft());
        encoderValueDriveRight = new ShuffleBoardInformation(tab, "encoder drive motor right",
                mDriveSubsystem.getEncoderRight());

        loadSlider = new ShuffleBoardInformation(tab, "Load motor speed", -1, 1, 0);
        transportSlider = new ShuffleBoardInformation(tab, "Transport motor speed", -1, 1, 0);
        shooterSlider = new ShuffleBoardInformation(tab, "Shoot motor speed", -1, 1, 0);
        pickUpSlider = new ShuffleBoardInformation(tab, "Pick up motor speed", -1, 1, 0);
        new ShuffleBoardInformation(tab, "Stop all ball subsystem motors", stopAllBallSubsystemsCommand);
        shooterSpeed = new ShuffleBoardInformation(tab, "Shooter Speed", mBallShooterSubsystem.getSpeed());
        new ShuffleBoardInformation(tab, "Shoot ball",
                new ShootBallCommand(getShooterSlider(), getLoadSlider(), getTransportSlider()));

        pose = new ShuffleBoardInformation(tab, "Robot Pose",
                "X: " + String.valueOf(mDriveSubsystem.getPose().getTranslation().getX()) + "\nY: "
                        + String.valueOf(mDriveSubsystem.getPose().getTranslation().getY())
               0         + "\nRotation(degrees): "
                        + String.valueOf(mDriveSubsystem.getPose().getRotation()));

        navxAngle = new ShuffleBoardInformation(tab, "Navx angle", mDriveSubsystem.getAngle());

        new ShuffleBoardInformation(tab, "Transport ball",
                new TransportBallCommand(Constants.standardTransportSpeed, false));
        transportSensor = new ShuffleBoardInformation(tab, "Transport Sensor",
                mBallTransportSubsystem.getSensor());

        angleToTarget = new ShuffleBoardInformation(tab, "Angle to target", (double) 0);
        new ShuffleBoardInformation(tab, "aim", aimCommand);
        new ShuffleBoardInformation(tab, "PID Tunnunig", aimCommand.getController());
        new ShuffleBoardInformation(tab, "Drive", testDriveCommand);
        new ShuffleBoardInformation(tab, "set Load Ball", loadBallCommmand);
        new ShuffleBoardInformation(tab, "set Shoot Ball", ballShooterCommand);
        new ShuffleBoardInformation(tab, "set transport Ball", transportBallcommand);
        new ShuffleBoardInformation(tab, "set Pick up motor", pickUpMotorCommand);
        new ShuffleBoardInformation(tab, "set Control Panel Motor",
        controlPanelMotorCommand);
    }

    public void update() {
        liftReed.update(mControlPanelSubsystem.getReedLiftBotom());
        bumperReed.update(mControlPanelSubsystem.getReedBumperFront());
        encoderValue.update(mControlPanelSubsystem.getEncoderValue());
        shooterSpeed.update(mBallShooterSubsystem.getSpeed());
        transportSensor.update(mBallTransportSubsystem.getSensor());
        encoderValueDriveLeft.update(mDriveSubsystem.getEncoderLeft());
        encoderValueDriveRight.update(mDriveSubsystem.getEncoderRight());
        Pose2d driveOdometry = mDriveSubsystem.getPose();
        pose.update(String.format("X: %f Y: %f Rot (deg): %f", driveOdometry.getTranslation().getX(),
                driveOdometry.getTranslation().getY(), driveOdometry.getRotation().getDegrees()));
        navxAngle.update(mDriveSubsystem.getAngle());
    }

    public DoubleSupplier getAngleToTarget() {
        return () -> angleToTarget.getDouble();
    }

    public static double getControlPanelMotorSlider() {
        return controlPanelMotorSlider.getSliderPosition();
    }

    public static double getDriveRotatePos() {
        return driveRotateSlider.getSliderPosition();
    }

    public static double getDriveForwardPos() {
        return driveForwardSlider.getSliderPosition();
    }

    public static void setDriveSlider(double forward, double rotate) {
        driveForwardSlider.setSliderPos(forward);
        driveRotateSlider.setSliderPos(rotate);
    }

    public static double getLoadSlider() {
        return loadSlider.getSliderPosition();
    }

    public static double getTransportSlider() {
        return transportSlider.getSliderPosition();
    }

    public static double getShooterSlider() {
        return shooterSlider.getSliderPosition();
    }

    public static double getPickUpSlider() {
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