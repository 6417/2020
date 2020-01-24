package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

public class TestRobotContainer extends RobotContainer {
    private static TestRobotContainer mInstance;
    private  NetworkTableEntry controlPanelMotorSlider;
    private NetworkTableEntry liftReed;
    private NetworkTableEntry bumperReed;
    private NetworkTableEntry endcoderValue;
    private final string t = "Test Mode"
    private ShuffleBoardInformation extendLiftButton;
    private ShuffleBoardInformation rejectLiftButton;
    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private ShuffleBoardInformation extendBumperButton;
    private ShuffleBoardInformation rejectBumperButton;
    private ShuffleBoardInformation speedSlider;
    private ShuffleBoardInformation encoderValue;
    
    

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
        
        extendLiftButton = new ShuffleBoardInformation(t ,"Extend ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        rejectLiftButton = new ShuffleBoardInformation(t, "Reject ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        liftReed = new ShuffleBoardInformation(t, "Reed switch of bottom Lift", ControlPanelSubsystem.getInstance().getReedLiftBotom());
        bumperReed = new ShuffleBoardInformation(t, "Reed switch of front Bumper", ControlPanelSubsystem.getInstance().getReedBumperFront());
        extendBumperButton = new ShuffleBoardInformation(t, "Extend ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        rejectBumperButton = new ShuffleBoardInformation(t, "Reject ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        speedSlider = new ShuffleBoardInformation(t, "ControlPanel Motor", -1, 1, 0);
        encoderValue = new ShuffleBoardInformation(t, "ControlPanel Motor Encoder", ControlPanelSubsystem.getInstance().getEndcoderValue());
    }

    public void update() {
        liftReed.update(ControlPanelSubsystem.getInstance().getReedLiftBotom());
        bumperReed.update(ControlPanelSubsystem.getInstance().getReedBumperFront());
        encoderValue.update(ControlPanelSubsystem.getInstance().getEndcoderValue());
    }

    public double getMotorSlider() {
        return speedSlider.getSliderPosition();
    }
}