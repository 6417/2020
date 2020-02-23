package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyVisionSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem mInstance;
    
    private DoubleSupplier distance;
    private DoubleSupplier angle;
    private DoubleSupplier offset;
    private BooleanSupplier target;

    protected VisionSubsystem() {
        constructor();
    }

    protected void constructor() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("vision");
    
        distance = () -> table.getEntry("Distance").getDouble(0);
        angle = () -> table.getEntry("Angle").getDouble(0);
        offset = () -> table.getEntry("XOffset").getDouble(0);
        target = () -> table.getEntry("Target").getBoolean(false);
    }

    public static VisionSubsystem getInstance() {
        if (Constants.VISION_SUBSYSTEM_ENABLED && mInstance == null) {
            mInstance = new VisionSubsystem();
        } else if (mInstance == null) {
            mInstance = new EmptyVisionSubsystem();
        }
        return mInstance;
    }

    /**
     * @return
     * true if a target has been detected
     */

    public boolean targetDetected() {
        return target.getAsBoolean();
    }

    public double getOffset() {
        return offset.getAsDouble();
    }

    public double getAngle() {
        return angle.getAsDouble();
    }

    public double getDistance() {
        return distance.getAsDouble();
    }
}