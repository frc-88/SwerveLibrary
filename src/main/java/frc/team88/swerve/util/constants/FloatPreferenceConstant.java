package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for float values.
 */
public final class FloatPreferenceConstant 
        extends BasePreferenceConstant<Float> {

    String name;
    float defaultValue;

    /**
     * Constructor. Will call update() once.
     * @param name The name to be used as a key in WPILib preferences
     * @param defaultValue The value that will be set as default if the value
     * doesn't exist in WPILib preferences
     */
    public FloatPreferenceConstant(String name, float defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Float getFromPreferences() {
        return Preferences.getInstance().getFloat(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Float value) {
        Preferences.getInstance().putFloat(name, value);
    }

}
