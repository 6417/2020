package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.ColorDetected;

public class GoToColorCommand extends CommandBase {
    private ControlPanelSubsystem mSubsystem = ControlPanelSubsystem.getInstance();
    private ColorDetected aimColor; 
    private ColorDetected cColor = mSubsystem.getColor();
    private ArrayList<ColorDetected> colors = Constants.colors;
    private int cColorIndex;
    private int aimColorIndex;

    public GoToColorCommand(ColorDetected aimColor) {
        this.aimColor = aimColor;
    }

    public int getIndex(ArrayList<ColorDetected> list, ColorDetected value) {
        int index = 0;
        for (int i = 0; i < list.size(); i++) {
            if (value == list.get(i)) {
                index = i;
            }
        }
        return index;
    }

    @Override
    public void initialize() {
        if (ControlPanelSubsystem.getInstance().getColor() != ColorDetected.NONE){       
             cColorIndex = getIndex(colors, cColor);
            Collections.rotate(colors, cColorIndex);

            aimColorIndex = getIndex(colors, aimColor);
        }
    }

    @Override
    public void execute() {
        if (ControlPanelSubsystem.getInstance().getColor() != ColorDetected.NONE) {
            this.initialize();
        }

        if (aimColorIndex <= 2) {
            mSubsystem.setMotor(-Constants.CONTROL_PANEL_MOTOR_SPEED);
        } else {
            mSubsystem.setMotor(Constants.CONTROL_PANEL_MOTOR_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return aimColor == mSubsystem.getColor();
    }
}