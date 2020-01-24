package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.PneumaticSubsystem.PneumaticState;

public class TestRobotContainer extends RobotContainer {
    private static TestRobotContainer mInstance;

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
        SmartDashboard.putData("Extend ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        SmartDashboard.putData("Reject ControlPanel Module", new PneumaticLiftCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        SmartDashboard.putData("Extend ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.FORWARD));
        SmartDashboard.putData("Reject ControlPanel Bumper", new PneumaticBumperCommand(PneumaticSubsystem.getInstance(), PneumaticState.REVERSE));
        // SmartDashboard.putBoolean("", value)
    }
}