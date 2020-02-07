package org.firstinspires.ftc.teamcode.Misc;

import org.openftc.revextensions2.ExpansionHubServo;

public class PositioningSystem {

    private ExpansionHubServo posLeft;
    private ExpansionHubServo posRight;
    private boolean attached = false;

    public PositioningSystem(ExpansionHubServo posLeft, ExpansionHubServo posRight) {
        this.posLeft = posLeft;
        this.posRight = posRight;
    }

    public void Detach() {
        posLeft.setPosition(0.9);
        posRight.setPosition(0.15);
        attached = false;
    }

    public void Attach() {
        posLeft.setPosition(0.55);
        posRight.setPosition(0.5);
        attached = true;
    }

    public boolean isAttached() { return attached; }

    public void Initial() {
        posLeft.setPosition(0.05);
        posRight.setPosition(1.0);
    }
}
