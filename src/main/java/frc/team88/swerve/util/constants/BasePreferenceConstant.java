package frc.team88.swerve.util.constants;

import java.util.Objects;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Base class of the PreferenceConstant interface.
 */
public abstract class BasePreferenceConstant<T> 
        implements PreferenceConstant<T> {

    private T value;
    private Consumer<T> changeHandler;

    /**
     * Get the value of this constant from WPILib Preferences.
     * @return The value from Preferences
     */
    protected abstract T getFromPreferences();

    /**
     * Set the value of this constant in WPILib Preferences.
     * @param value The value to set
     */
    protected abstract void setInPreferences(T value);

    @Override
    public final void update() {
        T newValue = this.getFromPreferences();
        boolean changed = !newValue.equals(this.value);
        this.value = newValue;
        if (Objects.nonNull(this.changeHandler) && changed) {
            this.changeHandler.accept(newValue);
        }
    }

    @Override
    public final T getValue() {
        return this.value;
    }

    @Override
    public final void assignChangeHandler(Consumer<T> handler) {
        this.changeHandler = handler;
    }

    @Override
    public final void setValue(T value) {
        this.setInPreferences(value);
        this.update();
    }
}