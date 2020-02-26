package org.firstinspires.ftc.teamcode.subsystems;

import org.openftc.revextensions2.ExpansionHubServo;

public class ClawSystem {
    private ExpansionHubServo claw;
    private ExpansionHubServo flipper;

    private boolean attached = false;
    private boolean lowered = false;

    public ClawSystem(ExpansionHubServo claw, ExpansionHubServo flipper1) {
        this.claw = claw;
        this.flipper = flipper1;
    }

    public void detach() {
        claw.setPosition(0.3);
        raiseFlipper();
        attached = false;
    }

    public void attach() {
        lowerFlipper();
        claw.setPosition(0.6);
        attached = true;
    }

    public void lowerFlipper() {
        lowered = true;
        flipper.setPosition(0);

    }

    public void raiseFlipper() {
        lowered = false;
        flipper.setPosition(1);
    }

    public void initial() {
        claw.setPosition(0.8);
    }

    public boolean isAttached() { return attached; }

    public boolean isLowered() { return lowered; }
}
