package org.firstinspires.ftc.teamcode.subsystems;

import org.openftc.revextensions2.ExpansionHubServo;

public class FoundationSystem {
    private ExpansionHubServo foundationLeft;
    private ExpansionHubServo foundationRight;
    private boolean attached = false;

    public FoundationSystem(ExpansionHubServo foundationLeft, ExpansionHubServo foundationRight) {
        this.foundationLeft = foundationLeft;
        this.foundationRight = foundationRight;
    }

    public void detach() {
        foundationLeft.setPosition(0);
        foundationRight.setPosition(1);
        attached = false;
    }

    public void attach() {
        foundationLeft.setPosition(1);
        foundationRight.setPosition(0);
        attached = true;
    }

    public boolean isAttached() { return attached; }
}
