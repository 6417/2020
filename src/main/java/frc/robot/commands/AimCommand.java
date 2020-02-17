package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import lombok.Setter;

public class AimCommand extends PIDCommand {
    @Setter
    private static double kP = 0;
    @Setter
    private static double kI = 0;
    @Setter
    private static double kD = 0;
    private static DoubleSupplier speed = () -> 0;

    // only Testing
    public AimCommand() {
        super(new PIDController(kP, kI, kD),
                () -> DriveSubsystem.getInstance().getAngle(),
                () -> 0,
                AimCommand::useOutput,
                DriveSubsystem.getInstance());
        getController().setTolerance(Constants.PID_TOLERANZ);
    }

    private static void useOutput(double out) {
        speed = () -> out;
        DriveSubsystem.getInstance().drive(0, out);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().stopDrive();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("P", null, (p) -> getController().setP(p));
        builder.addDoubleProperty("I", null, (i) -> getController().setI(i));
        builder.addDoubleProperty("D", null, (d) -> getController().setD(d));
        builder.addDoubleProperty("speed", speed, null);
        builder.addDoubleProperty("navx angle", () -> DriveSubsystem.getInstance().getAngle(), null);
    }
}