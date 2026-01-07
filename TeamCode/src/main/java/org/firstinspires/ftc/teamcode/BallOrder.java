package org.firstinspires.ftc.teamcode;

public enum BallOrder {
    // OBELISK april tags
    // 21 = GPP (green purple purple)
    // 22 = PGP (purple green purple)
    // 23 = PPG (purple purple green)

    // Our preload pattern PPG_23:
    // SPIN_P1 = purple (front)
    // SPIN_P2 = purple (front)
    // SPIN_P3 = green (through shooter)

    GPP_21(21),
    PGP_22(22),
    PPG_23(23);

    final int obeliskID;

    BallOrder(int obeliskID) {
        this.obeliskID = obeliskID;
    }

    public static BallOrder forObeliskID(int obeliskID) {
        for (BallOrder e : values()) {
            if (e.obeliskID == obeliskID) {
                return e;
            }
        }
        return BallOrder.PPG_23; // default
    }
}
