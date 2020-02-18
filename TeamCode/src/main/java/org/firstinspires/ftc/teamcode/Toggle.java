package org.firstinspires.ftc.teamcode;

public class Toggle {

    private boolean lastInput = false;
    private boolean pressed = false;
    private boolean changed = false;
    private boolean state = false;

    Toggle( ) { }
    Toggle( boolean initialState ) { this.state = initialState; }

    public void update(boolean input) {
        if (lastInput != input) {
            changed = true;
            pressed = input;
            if (pressed) state = !state;
            lastInput = input;
        } else {
            changed = false;
        }
    }

    public void set(boolean input) {
        if (lastInput != input) {
            state = input;
            changed = true;
        } else {
            changed = false;
        }
    }

    public boolean toggle() {
        state = !state;
        changed = true;
        return state;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean isChanged() {
        return changed;
    }

    public boolean getState() {
        return state;
    }

    public boolean isClicked() {
        return changed && pressed;
    }
}
