package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for string values.
 */
public final class StringPreferenceConstant extends BasePreferenceConstant<String> {

    String name;
    String defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name
     *                         The name to be used as a key in WPILib preferences
     * @param defaultValue
     *                         The value that will be set as default if the value
     *                         doesn't exist in WPILib preferences
     */
    public StringPreferenceConstant(String name, String defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected String getFromPreferences() {
        return Preferences.getInstance().getString(name, defaultValue);
    }

    @Override
    protected void setInPreferences(String value) {
        Preferences.getInstance().putString(name, value);
    }

}
