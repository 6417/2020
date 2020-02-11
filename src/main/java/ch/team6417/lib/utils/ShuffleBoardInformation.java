package ch.team6417.lib.utils;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleBoardInformation {

    private NetworkTableEntry information;
    private double defaultV;
    private boolean defaultBoolean;

    public ShuffleBoardInformation(String tab, String name, double information){
        String testname = name + "t";
        System.out.println(tab + " " + name + " " +  String.valueOf(information) + " " +  
                String.valueOf(Shuffleboard.getTab(tab)));//.add(testname, information)));//.getEntry()) + " " + this.information);

        try {
            this.information = Shuffleboard.getTab(tab)
                .add(name, information)
                .getEntry();
            System.out.println(this.information);
        } catch (NullPointerException e) {
            System.out.println(e.getMessage());
        }
        
    }

    public ShuffleBoardInformation(String tab, String name, Boolean information) {
        try {
            this.information = Shuffleboard.getTab(tab).add(name, information).getEntry();
        } catch (NullPointerException e) {
            System.out.println(e.getMessage());
        }
        
    }

    public ShuffleBoardInformation(String tab, String name, Sendable information) {
        try {
            System.out.println(name);
            Shuffleboard.getTab(tab).add(name, information);
        } catch (NullPointerException e) {
            System.out.println(e.getMessage());
        }
    }

    public ShuffleBoardInformation(String tab, String name, double min, double max, double defaultV) {
        this.defaultV = defaultV;
        try {
            information = Shuffleboard.getTab(tab).add(name, defaultV).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", min, "max", max)).getEntry();
        } catch (NullPointerException e) {
            System.out.println(e.getMessage());
        }
    }

    public ShuffleBoardInformation(String tab, String name, boolean information, boolean defaultV) {
        defaultBoolean = defaultV;
        try{
            this.information = Shuffleboard.getTab(tab).add(name, information).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        } catch (NullPointerException e) {
            System.out.println(e.getMessage());
        }
        
    }

    public ShuffleBoardInformation(String tab, String name, String information) {
        try{
            this.information = Shuffleboard.getTab(tab).add(name, information).getEntry();
        } catch (NullPointerException e){
            System.out.println(e.getMessage());
        }
    }

    public void update(boolean value) {
        if (this.information != null) {
            information.setBoolean(value);
        }
    }

    public void update(double value) {
        if (this.information != null) {
            information.setDouble(value);
        }
    }

    public void update(String value) {
        if (this.information != null) {
            information.setString(value);
        }
    }

    public double getSliderPosition() throws IllegalArgumentException {
        if (this.information != null) {
            return information.getDouble(defaultV);
        } else {
            throw new IllegalArgumentException("This is a button or not a slider");
        }
    }

    public void setSliderPos(double value) {
        information.setDouble(value);
    }

    public boolean getButtonState() {
        return information.getBoolean(defaultBoolean);
    }

    public double getDouble() {
        return information.getDouble(0);
    }
}