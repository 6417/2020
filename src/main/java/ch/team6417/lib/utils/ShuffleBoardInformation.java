public class ShuffleBoardInformation(){

    private Networktable information;
    private double defaultValue;

    public ShuffleBoardInformation(String tab, String name, Sendable information){
        this.object = information;
        this.information = ShuffleBoard.getTab(tab)
        .add(name, information)
        .getEntry();

    }

    public ShuffleBoardInformation(String tab, String name, double min, double max, double default){
        this.defaultValue = default;
        information = Shuffleboard.getTab(tab)
        .add(name, default)
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
        if (this.infomation != null){
            information.setDouble(value);
        }

    public double getSliderPosition(){
        if (this.information != null){
            return information.getDouble(defaultValue);
        }
    }

    
}