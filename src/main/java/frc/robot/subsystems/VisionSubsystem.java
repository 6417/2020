package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyVisionSubsystem;
import lombok.Getter;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem mInstance;
    private NetworkTableEntry Distance;
    private NetworkTableEntry Angle;
    private NetworkTableEntry Offset;
    private NetworkTableEntry Target;
    
    @Getter
    private double distance;
    @Getter
    private double angle;
    @Getter
    private double offset;
    private boolean target;

    protected VisionSubsystem() {
        constructor();
    }

    protected void constructor() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("vision");
    
        Distance = table.getEntry("Distance");
        Angle = table.getEntry("Angle");
        Offset = table.getEntry("XOffset");
        Target = table.getEntry("Target");
    }

    @Override
    public void periodic() {
        distance = Distance.getDouble(0);
        angle = Angle.getDouble(0);
        offset = Offset.getDouble(0);
        target = Target.getBoolean(false);
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
        return target;
    }
}