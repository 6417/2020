package ch.team6417.lib.utils;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleBoardInformation {

    private NetworkTableEntry information;
    private double defaultValue;

    public ShuffleBoardInformation(String tab, String name, Double information){
        this.information = Shuffleboard.getTab(tab)
                .add(name, information)
                .getEntry();
    }

    public ShuffleBoardInformation(String tab, String name, Boolean information) {
        this.information = Shuffleboard.getTab(tab)
                .add(name, information)
                .getEntry();
    }

    public ShuffleBoardInformation(String tab, String name, Sendable information) {
        Shuffleboard.getTab(tab).add(name, information);
    }

    public ShuffleBoardInformation(String tab, String name, double min, double max, double defaultV){
        this.defaultValue = defaultV;
        information = Shuffleboard.getTab(tab)
                .add(name, defaultV)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", min, "max", max))
                .getEntry();        
    }

    public void update(boolean value){
        if (this.information != null){
            information.setBoolean(value);
        }
    }

    public void update(double value){
        if (this.information != null){
            information.setDouble(value);
        }
    }

    public double getSliderPosition() throws IllegalArgumentException {
        if (this.information != null){
            return information.getDouble(defaultValue);
        } else {
            throw new IllegalArgumentException("ther is no Slider");
        }
    }    
}