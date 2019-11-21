package org.firstinspires.ftc.teamcode;

public class Toggle {

    private boolean last = false;
    private boolean pressed = false;
    private boolean state = false;

    Toggle( boolean initialState ) {
        this.state = initialState;
    }

    public void update(boolean input){
        if (last != input)
            pressed = input;
        else
            pressed = false;

        if (pressed)
            state = !state;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean getState() {
        return state;
    }
}
