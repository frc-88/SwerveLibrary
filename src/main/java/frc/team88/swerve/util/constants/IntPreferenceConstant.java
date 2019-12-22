package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for int values.
 */
public final class IntPreferenceConstant extends BasePreferenceConstant<Integer> {

    String name;
    int defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name         The name to be used as a key in WPILib preferences
     * @param defaultValue The value that will be set as default if the value
     *                     doesn't exist in WPILib preferences
     */
    public IntPreferenceConstant(String name, int defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Integer getFromPreferences() {
        return Preferences.getInstance().getInt(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Integer value) {
        Preferences.getInstance().putInt(name, value);
    }

}
