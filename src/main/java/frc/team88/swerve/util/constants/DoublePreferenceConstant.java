package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for double values
 */
public class DoublePreferenceConstant extends BasePreferenceConstant<Double> {

    String name;
    double defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name
     *                         The name to be used as a key in WPILib preferences
     * @param defaultValue
     *                         The value that will be set as default if the value
     *                         doesn't exist in WPILib preferences
     */
    public DoublePreferenceConstant(String name, double defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Double getFromPreferences() {
        return Preferences.getInstance().getDouble(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Double value) {
        Preferences.getInstance().putDouble(name, value);
    }

}
