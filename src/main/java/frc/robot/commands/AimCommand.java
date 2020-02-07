package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AimCommand extends PIDCommand {
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static DoubleSupplier speed = () -> 0;
    private static DriveCommand driveCommand = new DriveCommand(() -> 0, speed);

    // only Testing
    public AimCommand() {
        super(new PIDController(kP, kI, kD), 
                TestRobotContainer.getInstance().getAngleToTarget(),
                () -> 0,
                AimCommand::useOutput,
                DriveSubsystem.getInstance());
        getController().setTolerance(3);
    }

    private static void useOutput(double out) {
        speed = () -> out;
        driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}