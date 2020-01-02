package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for boolean values.
 */
public class BooleanPreferenceConstant extends BasePreferenceConstant<Boolean> {

    String name;
    boolean defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name
     *                         The name to be used as a key in WPILib preferences
     * @param defaultValue
     *                         The value that will be set as default if the value
     *                         doesn't exist in WPILib preferences
     */
    public BooleanPreferenceConstant(String name, boolean defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Boolean getFromPreferences() {
        return Preferences.getInstance().getBoolean(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Boolean value) {
        Preferences.getInstance().putBoolean(name, value);
    }

}
