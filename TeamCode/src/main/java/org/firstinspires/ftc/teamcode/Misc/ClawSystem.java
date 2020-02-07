package org.firstinspires.ftc.teamcode.Misc;

import org.openftc.revextensions2.ExpansionHubServo;

public class ClawSystem {
    private ExpansionHubServo clawRight;
    private ExpansionHubServo clawLeft;
    private ExpansionHubServo flipper;

    private boolean attached = false;
    private boolean lowered = false;

    public ClawSystem(ExpansionHubServo clawLeft, ExpansionHubServo clawRight, ExpansionHubServo flipper1) {
        this.clawLeft = clawLeft;
        this.clawRight = clawRight;
        this.flipper = flipper1;
    }

    public void Detach() {
        clawRight.setPosition(1);
        clawLeft.setPosition(0);
        attached = false;
    }

    public void Attach() {
        clawRight.setPosition(0.15);
        clawLeft.setPosition(1);
        attached = true;
    }

    public void lowerFlipper() {
        lowered = true;
        flipper.setPosition(0.5);

    }

    public void raiseFlipper() {
        lowered = false;
        flipper.setPosition(1);
    }

    public boolean isAttached() { return attached; }

    public boolean isLowered() { return lowered; }
}
