package org.firstinspires.ftc.teamcode.misc;

public class ButtonToggle {

    private boolean lastState;
    private boolean toggle;

    public boolean getState(boolean state) {
        if (state && !lastState)
            toggle = !toggle;
        lastState = state;
        return toggle;
    }

}
