package org.firstinspires.ftc.teamcode.misc;

public class ButtonToggle {

    private boolean lastState;
    private boolean toggle;

    public boolean getState(boolean state) {
        lastState = state;

        if (state && !lastState)
            return true;
        else
            return false;
    }

}