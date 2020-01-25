package frc.robot;

import java.util.Map;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.commands.TestCommands.TurnRightCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

public class TestRobotContainer {
    private static TestRobotContainer mInstance;
    private final String tab = "Test Mode";
    private ShuffleBoardInformation extendLiftButton;
    private ShuffleBoardInformation rejectLiftButton;
    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private ShuffleBoardInformation extendBumperButton;
    private ShuffleBoardInformation rejectBumperButton;
    private ShuffleBoardInformation speedSlider;
    private ShuffleBoardInformation encoderValue;
    private ShuffleBoardInformation driveLeftSlider;
    private ShuffleBoardInformation driveRightSlider;
    
    

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
        
        extendLiftButton = new ShuffleBoardInformation(tab, "Extend ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        rejectLiftButton = new ShuffleBoardInformation(tab, "Reject ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        liftReed = new ShuffleBoardInformation(tab, "Reed switch of bottom Lift", ControlPanelSubsystem.getInstance().getReedLiftBotom());
        bumperReed = new ShuffleBoardInformation(tab, "Reed switch of front Bumper", ControlPanelSubsystem.getInstance().getReedBumperFront());
        extendBumperButton = new ShuffleBoardInformation(tab, "Extend ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        rejectBumperButton = new ShuffleBoardInformation(tab, "Reject ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        speedSlider = new ShuffleBoardInformation(tab, "ControlPanel Motor", -1, 1, 0);
        encoderValue = new ShuffleBoardInformation(tab, "ControlPanel Motor Encoder", Double.valueOf(ControlPanelSubsystem.getInstance().getEncoderValue()));
        driveRightSlider = new ShuffleBoardInformation(tab, "Turn right", -1, 1, 0);
        driveLeftSlider = new ShuffleBoardInformation(tab, "Turn left", -1, 1, 0);
        new ShuffleBoardInformation(tab, "Stop tankdrive", new StopTankDriveCommand());
    }

    public void update() {
        liftReed.update(ControlPanelSubsystem.getInstance().getReedLiftBotom());
        bumperReed.update(ControlPanelSubsystem.getInstance().getReedBumperFront());
        encoderValue.update(ControlPanelSubsystem.getInstance().getEncoderValue());
    }

    public double getMotorSlider() {
        return speedSlider.getSliderPosition();
    }

    public double getDriveRightPos() {
        return driveRightSlider.getSliderPosition();
    }

    public double getDriveLeftPos() {
        return driveLeftSlider.getSliderPosition();
    }
}