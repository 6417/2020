package frc.robot;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import ch.team6417.lib.utils.ShuffleBoardInformation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BallLoaderCommand;
import frc.robot.commands.BallPickupMotorCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtendPickupModuleCommand;
import frc.robot.commands.RetractPickupModuleCommand;
import frc.robot.commands.SetMotorForRotationsCommand;
import frc.robot.commands.PneumaticLiftCommand;
import frc.robot.commands.PneumaticBumperCommand;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.TransportBallCommand;
import frc.robot.commands.TestCommands.StopAllBallSubsystemsCommand;
import frc.robot.commands.TestCommands.StopTankDriveCommand;
import frc.robot.subsystems.BallPickUpSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.BallTransportSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;


public class TestRobotContainer {
    private static TestRobotContainer mInstance;
    private final String tab = "Test Mode";
    private final String ControlPanelTab = "ControlPanel Test";
    private final String BallSubsystemsTab = "BallSubsystems Test";
    private final String DriveSubsystemsTab = "DriveSubsystem Test";

    private BallPickUpSubsystem mBallPickUpSubsystem;
    private BallShooterSubsystem mBallShooterSubsystem;
    private BallTransportSubsystem mBallTransportSubsystem;
    private ControlPanelSubsystem mControlPanelSubsystem;
    private DriveSubsystem mDriveSubsystem;
    private PneumaticSubsystem mPneumaticSubsystem;

    private ShuffleBoardInformation liftReed;
    private ShuffleBoardInformation bumperReed;
    private static ShuffleBoardInformation controlPanelMotorSlider;
    private ShuffleBoardInformation encoderValue;
    private ShuffleBoardInformation driveForwardSlider;
    private ShuffleBoardInformation driveRotateSlider;
    private ShuffleBoardInformation loadSlider;
    private ShuffleBoardInformation shooterSlider;
    private ShuffleBoardInformation transportSlider;
    private ShuffleBoardInformation pickUpSlider;
    private ShuffleBoardInformation shooterSpeed;
    private ShuffleBoardInformation transportSensor;
    private ShuffleBoardInformation encoderValueDriveLeft;
    private ShuffleBoardInformation encoderValueDriveRight;
    private ShuffleBoardInformation pose;
    private ShuffleBoardInformation navxAngle;
    private ShuffleBoardInformation angleToTarget;
    private ShuffleBoardInformation rawPose;
    private AimCommand aimCommand;
    private AHRS navx;
    private DriveCommand testDriveCommand;
    private BallLoaderCommand loadBallCommmand;
    private BallShooterCommand ballShooterCommand;
    private TransportBallCommand transportBallcommand;
    private BallPickupMotorCommand pickUpMotorCommand;
    private CommandBase controlPanelMotorCommand;
    private StopAllBallSubsystemsCommand stopAllBallSubsystemsCommand;
    private StopTankDriveCommand stopTankDriveCommand;
    private ExtendPickupModuleCommand extendPickupModuleCommand;
    private RetractPickupModuleCommand retractPickupModuleCommand;

    private TestRobotContainer() {

    }

    public static TestRobotContainer getInstance() {
        if (mInstance == null) {
            mInstance = new TestRobotContainer();

            // initalize mSubsystems

            mInstance.mBallPickUpSubsystem = BallPickUpSubsystem.getInstance();
            mInstance.mBallShooterSubsystem = BallShooterSubsystem.getInstance();
            mInstance.mBallTransportSubsystem = BallTransportSubsystem.getInstance();
            mInstance.mControlPanelSubsystem = ControlPanelSubsystem.getInstance();
            mInstance.mDriveSubsystem = DriveSubsystem.getInstance();

            // initialize Commands

            mInstance.testDriveCommand = new DriveCommand(() -> mInstance.getDriveForwardPos(),
                    () -> mInstance.getDriveRotatePos());
            mInstance.loadBallCommmand = new BallLoaderCommand(() -> mInstance.getLoadSlider());
            mInstance.ballShooterCommand = new BallShooterCommand(() -> mInstance.getShooterSlider(), false);
            mInstance.transportBallcommand = new TransportBallCommand(() -> mInstance.getTransportSlider(), false);
            mInstance.pickUpMotorCommand = new BallPickupMotorCommand(() -> mInstance.getPickUpSlider());
            mInstance.aimCommand = new AimCommand();
            mInstance.extendPickupModuleCommand = new ExtendPickupModuleCommand();
            mInstance.retractPickupModuleCommand = new RetractPickupModuleCommand();
            mInstance.controlPanelMotorCommand = new CommandBase() {
                @Override
                public void execute() {
                    mInstance.mControlPanelSubsystem.setMotor(() -> mInstance.getControlPanelMotorSlider());
                }
            };
            mInstance.stopAllBallSubsystemsCommand = new StopAllBallSubsystemsCommand();
            mInstance.stopTankDriveCommand = new StopTankDriveCommand();

            TestRobotContainer.showOnShuffleBoard();

            return mInstance;
        } else {
            return mInstance;
        }
    }

    private static void showOnShuffleBoard() {

        /**
         * Informtions displayed on the ControlPanel Tab
         */

            /**
             * Commands
             */

        new ShuffleBoardInformation(mInstance.ControlPanelTab, "Extend ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.FORWARD));
        new ShuffleBoardInformation(mInstance.ControlPanelTab, "Reject ControlPanel Module",
                new PneumaticLiftCommand(PneumaticState.REVERSE));
        new ShuffleBoardInformation(mInstance.ControlPanelTab, "Extend Bumper Command", 
                new PneumaticBumperCommand(PneumaticState.FORWARD));
        new ShuffleBoardInformation(mInstance.ControlPanelTab, "Retract Bumper Command",
                new PneumaticBumperCommand(PneumaticState.REVERSE));
        new ShuffleBoardInformation(mInstance.ControlPanelTab, "set Control Panel Motor", mInstance.controlPanelMotorCommand);
        new ShuffleBoardInformation(mInstance.ControlPanelTab, "stop Control Panel Motor", new SetMotorForRotationsCommand(0));
            /**
             * Getters
             */

        mInstance.liftReed = new ShuffleBoardInformation(mInstance.ControlPanelTab, "Reed switch of bottom Lift",
                mInstance.mControlPanelSubsystem.getReedLiftBotom());
        mInstance.bumperReed = new ShuffleBoardInformation(mInstance.ControlPanelTab, "Reed switch of front Bumper",
                mInstance.mControlPanelSubsystem.getReedBumperFront());
        mInstance.controlPanelMotorSlider = new ShuffleBoardInformation(mInstance.ControlPanelTab, "ControlPanelMotor", -1, 1, 0);
        mInstance.encoderValue = new ShuffleBoardInformation(mInstance.ControlPanelTab, "ControlPanel Motor Encoder",
                mInstance.mControlPanelSubsystem.getEncoderValue());

        /**
         * Informations displayed on the DriveSubsystem Tab
         */
            
            /**
             * Sliders
             */

            mInstance.driveRotateSlider = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Rotatr Tank", -1, 1, 0);
            mInstance.driveForwardSlider = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Drive Forward", -1, 1, 0);

            /**
             * Commands
             */

        new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Stop tankdrive", mInstance.stopTankDriveCommand);
        new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "aim", mInstance.aimCommand);
        new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "PID Tunnunig", mInstance.aimCommand.getController());
        new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Drive", mInstance.testDriveCommand);
    

            /**
             * Getters
             */

        mInstance.encoderValueDriveLeft = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "encoder drive motor left",
        mInstance.mDriveSubsystem.getEncoderLeft());
        mInstance.encoderValueDriveRight = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "encoder drive motor right",
            mInstance.mDriveSubsystem.getEncoderRight());
        mInstance.pose = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Robot Pose",
            "X: " + String.valueOf(mInstance.mDriveSubsystem.getPose().getTranslation().getX()) + "\nY: "
            + String.valueOf(mInstance.mDriveSubsystem.getPose().getTranslation().getY())
            + "\nRotation(degrees): " + String.valueOf(mInstance.mDriveSubsystem.getPose().getRotation()));
            mInstance.angleToTarget = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Angle to target", (double) 0);
        mInstance.rawPose = new ShuffleBoardInformation(mInstance.DriveSubsystemsTab, "Raw Robot Pose", DriveSubsystem.getInstance().getPose().toString());
        
        /**
         * Informations displayed on the BallSubsystems Tab
         */
        
            /**
             * Sliders
             */

        mInstance.loadSlider = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Load motor speed", -1, 1, 0);
        mInstance.transportSlider = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Transport motor speed", -1, 1, 0);
        mInstance.shooterSlider = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Shoot motor speed", -1, 1, 0);
        mInstance.pickUpSlider = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Pick up motor speed", -1, 1, 0);

            /**
             * Commands
             */

        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Stop all ball subsystem motors",
            mInstance.stopAllBallSubsystemsCommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Shoot ball", new ShootBallCommand(mInstance.getShooterSlider(),
            mInstance.getLoadSlider(), mInstance.getTransportSlider()));
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Transport ball",
            mInstance.transportBallcommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "set Load Ball", mInstance.loadBallCommmand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "set Shoot Ball", mInstance.ballShooterCommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "set transport Ball", mInstance.transportBallcommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "set Pick up motor", mInstance.pickUpMotorCommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "extendPickupModule", mInstance.extendPickupModuleCommand);
        new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "retractPickupModule", mInstance.retractPickupModuleCommand);
        

            /**
             * Getters
             */

        mInstance.shooterSpeed = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Shooter Speed",
            mInstance.mBallShooterSubsystem.getSpeed());
        mInstance.navxAngle = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Navx angle",
            mInstance.mDriveSubsystem.getAngle());
        mInstance.transportSensor = new ShuffleBoardInformation(mInstance.BallSubsystemsTab, "Transport Sensor",
            mInstance.mBallTransportSubsystem.getSensor());
    }


    public void update() {
        mInstance.liftReed.update(mInstance.mControlPanelSubsystem.getReedLiftBotom());
        mInstance.bumperReed.update(mInstance.mControlPanelSubsystem.getReedBumperFront());
        mInstance.encoderValue.update(mInstance.mControlPanelSubsystem.getEncoderValue());
        mInstance.shooterSpeed.update(mInstance.mBallShooterSubsystem.getSpeed());
        mInstance.transportSensor.update(mInstance.mBallTransportSubsystem.getSensor());
        mInstance.encoderValueDriveLeft.update(mInstance.mDriveSubsystem.getEncoderLeft());
        mInstance.encoderValueDriveRight.update(mInstance.mDriveSubsystem.getEncoderRight());
        Pose2d driveOdometry = mInstance.mDriveSubsystem.getPose();
        mInstance.pose.update(String.format("X: %f Y: %f Rot (deg): %f", driveOdometry.getTranslation().getX(),
                driveOdometry.getTranslation().getY(), driveOdometry.getRotation().getDegrees()));
        mInstance.navxAngle.update(mInstance.mDriveSubsystem.getAngle());
        mInstance.rawPose.update(DriveSubsystem.getInstance().getPose().toString());
        
        
        mInstance.ballShooterCommand.schedule(true);
        mInstance.loadBallCommmand.schedule(true);
        mInstance.transportBallcommand.schedule(true);
        mInstance.pickUpMotorCommand.schedule(true);
    }

    /**
     * Getters
     */

    public DoubleSupplier getAngleToTarget() {
        return () -> mInstance.angleToTarget.getDouble();
    }

    public double getControlPanelMotorSlider() {
        return mInstance.controlPanelMotorSlider.getSliderPosition();
    }

    public double getDriveRotatePos() {
        return mInstance.driveRotateSlider.getSliderPosition();
    }

    public double getDriveForwardPos() {
        return mInstance.driveForwardSlider.getSliderPosition();
    }

    public void setDriveSlider(double forward, double rotate) {
        mInstance.driveForwardSlider.setSliderPos(forward);
        mInstance.driveRotateSlider.setSliderPos(rotate);
    }

    public double getLoadSlider() {
        return mInstance.loadSlider.getSliderPosition();
    }

    public double getTransportSlider() {
        return mInstance.transportSlider.getSliderPosition();
    }

    public double getShooterSlider() {
        return mInstance.shooterSlider.getSliderPosition();
    }

    public double getPickUpSlider() {
        return mInstance.pickUpSlider.getSliderPosition();
    }

    public void setShooterSliderPos(double pos) {
        mInstance.shooterSlider.setSliderPos(pos);
    }

    public void setLoadSliderPos(double pos) {
        mInstance.loadSlider.setSliderPos(pos);
    }

    public void setTransportSliderPos(double pos) {
        mInstance.transportSlider.setSliderPos(pos);
    }

    public void setPickUpSliderPos(double pos) {
        mInstance.pickUpSlider.setSliderPos(pos);
    }

    public void setControlPanelMotorSlider(double pos){
        mInstance.controlPanelMotorSlider.setSliderPos(pos);
    }

    public double getAngle() {
        return mDriveSubsystem.getAngle();
    }
}