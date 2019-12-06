package org.firstinspires.ftc.teamcode;

public class Toggle {

    private boolean last = false;
    private boolean pressed = false;
    private boolean changed = false;
    private boolean state = false;

    Toggle( ) { }
    Toggle( boolean initialState ) { this.state = initialState; }

    public void update(boolean input) {
        if (last != input) {
            changed = true;
            pressed = input;
            if (pressed) state = !state;
            last = input;
        } else {
            changed = false;
        }
    }

    public void set(boolean input) {
        if (last != input) {
            state = input;
            changed = true;
        } else {
            changed = false;
        }
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
}
