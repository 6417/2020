package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.SparkMax;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import frc.robot.commands.BallShooterGroup;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.TestCommands.StopAllBallSubsystemsCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;

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
    private ShuffleBoardInformation shooterCommandGroup;

    private TestRobotContainer() {
        showOnShuffleBoard();
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

        new ShuffleBoardInformation(tab, "Extend ControlPanel Bumper",
                new PneumaticBumperCommand(PneumaticState.FORWARD));
        new ShuffleBoardInformation(tab, "Reject ControlPanel Bumper",
                new PneumaticBumperCommand(PneumaticState.REVERSE));

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

        loadSlider = new ShuffleBoardInformation(tab, "Load motor speed", -1, 1, 0);
        transportSlider = new ShuffleBoardInformation(tab, "Transport motor speed", -1, 1, 0);
        shooterSlider = new ShuffleBoardInformation(tab, "Shoot motor speed", -1, 1, 0);
        pickUpSlider = new ShuffleBoardInformation(tab, "Pick up motor speed", -1,  1, 0);
        new ShuffleBoardInformation(tab, "Stop all ball subsystem motors", new StopAllBallSubsystemsCommand());
        shooterSpeed = new ShuffleBoardInformation(tab, "Shooter Speed", BallShooterSubsystem.getInstance().getSpeed());
        shooterCommandGroup = new ShuffleBoardInformation(tab, "test command Group", new BallShooterGroup());
    }

    public void update() {
        liftReed.update(ControlPanelSubsystem.getInstance().getReedLiftBotom());            
        bumperReed.update(ControlPanelSubsystem.getInstance().getReedBumperFront());
        encoderValue.update(ControlPanelSubsystem.getInstance().getEncoderValue());
        shooterSpeed.update(BallShooterSubsystem.getInstance().getSpeed());
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