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
        Shuffleboard.getTab("Test Mode").add("Extend ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        Shuffleboard.getTab("Test Mode").add("Reject ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        liftReed = Shuffleboard.getTab("Test Mode").add("Reed switch of botom Lift", ControlPanelSubsystem.getInstance().getReedLiftBotom()).getEntry();
        bumperReed = Shuffleboard.getTab("Test Mode").add("Reed switch of front Bumper", ControlPanelSubsystem.getInstance().getReedBumperFront()).getEntry();
        Shuffleboard.getTab("Test Mode").add("Extend ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        Shuffleboard.getTab("Test Mode").add("Reject ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
               
        controlPanelMotorSlider = Shuffleboard.getTab("Test Mode")
        .add("ControlPanel Motor", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

        endcoderValue = Shuffleboard.getTab("Test Mode").add("Control panel motor Endcoder", ControlPanelSubsystem.getInstance().getEndcoderValue()).getEntry();
        // Shuffleboard.getTab("Test Mode").add("Compressor", PneumaticSubsystem.getInstance().initSendable(new SendableBuilder()));
    }

    public void update() {
        liftReed.setBoolean(ControlPanelSubsystem.getInstance().getReedLiftBotom());
        bumperReed.setBoolean(ControlPanelSubsystem.getInstance().getReedBumperFront());
        endcoderValue.setDouble(ControlPanelSubsystem.getInstance().getEndcoderValue());
    }

    public double getMotorSlider() {
        return controlPanelMotorSlider.getDouble(0);
    }
}