package frc.robot;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

public class TestRobotContainer {
    private static TestRobotContainer mInstance;
    private final String tab = "Test Mode";
    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private ShuffleBoardInformation controlPanelMotorSlider;
    private ShuffleBoardInformation encoderValue;
    private static ShuffleBoardInformation driveLeftSlider;
    private static ShuffleBoardInformation driveRightSlider;


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

        // new ShuffleBoardInformation(tab, "Extend ControlPanel Module", new
        // PneumaticLiftCommand(PneumaticSubsystem.getInstance(),
        // PneumaticState.FORWARD));
        // new ShuffleBoardInformation(tab, "Reject ControlPanel Module", new
        // PneumaticLiftCommand(PneumaticSubsystem.getInstance(),
        // PneumaticState.REVERSE));
        // liftReed = new ShuffleBoardInformation(tab, "Reed switch of bottom Lift",
        // ControlPanelSubsystem.getInstance().getReedLiftBotom());

        // new ShuffleBoardInformation(tab, "Extend ControlPanel Bumper", new
        // PneumaticBumperCommand(PneumaticSubsystem.getInstance(),
        // PneumaticState.FORWARD));
        // new ShuffleBoardInformation(tab, "Reject ControlPanel Bumper", new
        // PneumaticBumperCommand(PneumaticSubsystem.getInstance(),
        // PneumaticState.REVERSE));
        // bumperReed = new ShuffleBoardInformation(tab, "Reed switch of front Bumper",
        // ControlPanelSubsystem.getInstance().getReedBumperFront());

        // controlPanelMotorSlider = new ShuffleBoardInformation(tab, "ControlPanel
        // Motor", -1, 1, 0);
        // encoderValue = new ShuffleBoardInformation(tab, "ControlPanel Motor Encoder",
        // Double.valueOf(ControlPanelSubsystem.getInstance().getEncoderValue()));
        driveRightSlider = new ShuffleBoardInformation(tab, "Turn right", -1, 1, 0);
        driveLeftSlider = new ShuffleBoardInformation(tab, "Turn left", -1, 1, 0);
        new ShuffleBoardInformation(tab, "Stop tankdrive", new StopTankDriveCommand());
    }

    public void update() {
        // liftReed.update(ControlPanelSubsystem.getInstance().getReedLiftBotom());
        // bumperReed.update(ControlPanelSubsystem.getInstance().getReedBumperFront());
        // encoderValue.update(ControlPanelSubsystem.getInstance().getEncoderValue());
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
        driveLeftSlider.setSlierPos(left);
        driveRightSlider.setSlierPos(right);
    }


}