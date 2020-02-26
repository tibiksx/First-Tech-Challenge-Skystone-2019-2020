package org.firstinspires.ftc.teamcode.subsystems;

import org.openftc.revextensions2.ExpansionHubServo;

public class PositioningSystem {

    private ExpansionHubServo posLeft;
    private ExpansionHubServo posRight;
    private boolean attached = false;
    private boolean initial = false;

    public PositioningSystem(ExpansionHubServo posLeft, ExpansionHubServo posRight) {
        this.posLeft = posLeft;
        this.posRight = posRight;
    }

    public void detach() {
        posLeft.setPosition(0.9);
        posRight.setPosition(0.15);
        attached = false;
        initial = false;
    }

    public void attach() {
        posLeft.setPosition(0.55);
        posRight.setPosition(0.5);
        attached = true;
        initial = false;
    }

    public boolean isAttached() { return attached; }

    public void initial() {
        initial = true;
        attached = false;
        posLeft.setPosition(0.05);
        posRight.setPosition(1.0);
    }

    public boolean isInitial() { return initial; }
}
