package frc.team88.swerve.util.constants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for long values.
 */
public final class LongPreferenceConstant extends BasePreferenceConstant<Long> {

    String name;
    long defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name
     *                         The name to be used as a key in WPILib preferences
     * @param defaultValue
     *                         The value that will be set as default if the value
     *                         doesn't exist in WPILib preferences
     */
    public LongPreferenceConstant(String name, long defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.getInstance().containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Long getFromPreferences() {
        return Preferences.getInstance().getLong(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Long value) {
        Preferences.getInstance().putLong(name, value);
    }

}
