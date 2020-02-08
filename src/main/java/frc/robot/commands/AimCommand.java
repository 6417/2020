package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.TestRobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import lombok.Getter;
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
                () -> TestRobotContainer.getInstance().getAngle(),
                () -> 0,
                AimCommand::useOutput,
                DriveSubsystem.getInstance());
        getController().setTolerance(3);
        System.out.println("aim command constructor");
    }

    private static void useOutput(double out) {
        speed = () -> out;
        DriveSubsystem.getInstance().drive(0, out);
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("aim command initialized");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void schedule() {
        super.schedule(false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("P", null, (p) -> getController().setP(p));
        builder.addDoubleProperty("I", null, (i) -> getController().setI(i));
        builder.addDoubleProperty("D", null, (d) -> getController().setD(d));
        builder.addDoubleProperty("speed", speed, null);
        builder.addDoubleProperty("navx angle", () -> TestRobotContainer.getInstance().getAngle(), null);
    }
}