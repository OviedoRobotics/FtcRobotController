package org.firstinspires.ftc.teamcode;


import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.teamcode.AutonomousBase.getObeliskShootOrder;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.Ball.Green;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.Ball.None;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.Ball.Purple;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P1;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P2;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P3;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class ShootOrderTest {

    // -------------------------------------------------------------------------
    // distanceTo helper
    // -------------------------------------------------------------------------

    @Test
    public void testMoveCost_sameSlot_isZero() {
        assertEquals(0, SPIN_P1.distanceTo(SPIN_P1));
        assertEquals(0, SPIN_P2.distanceTo(SPIN_P2));
        assertEquals(0, SPIN_P3.distanceTo(SPIN_P3));
    }

    @Test
    public void testMoveCost_adjacentSlots_isOne() {
        assertEquals(1, SPIN_P1.distanceTo(SPIN_P2));
        assertEquals(1, SPIN_P2.distanceTo(SPIN_P1));
        assertEquals(1, SPIN_P2.distanceTo(SPIN_P3));
        assertEquals(1, SPIN_P3.distanceTo(SPIN_P2));
    }

    @Test
    public void testMoveCost_nonAdjacentSlots_isTwo() {
        assertEquals(2, SPIN_P1.distanceTo(SPIN_P3));
        assertEquals(2, SPIN_P3.distanceTo(SPIN_P1));
    }

    // -------------------------------------------------------------------------
    // Edge cases
    // -------------------------------------------------------------------------

    @Test
    public void testAllEmpty_returnsEmptyArray() {
        HardwareSwyftBot.SpindexerState[] result = getObeliskShootOrder(BallOrder.PPG_23, SPIN_P2, None, None, None);
        assertEquals(0, result.length);
    }

    @Test
    public void testSingleOccupied_returnsThatSlot() {
        // Only P2 has a ball; result must be exactly [P2]
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P1, None, Green, None));
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P1, Purple, None, None));
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, None, None, Purple));
    }

    // -------------------------------------------------------------------------
    // Two occupied slots — color matching
    // -------------------------------------------------------------------------

    @Test
    public void testTwoSlots_greenFirstMatches() {
        // P1=Purple, P2=empty, P3=Green; obelisk GPP_21 expects [G, P, P]
        // [P3, P1] → G, P → 2/2 color matches  (movement from P2: 1+2=3)
        // [P1, P3] → P, G → 0/2 color matches
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3, SPIN_P1},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P2, Purple, None, Green));
    }

    @Test
    public void testTwoSlots_purpleFirstMatches() {
        // P1=empty, P2=Purple, P3=Green; obelisk PPG_23 expects [P, P, G]
        // [P2, P3] → P, G → 1/2 matches, cost from P1: 1+1=2
        // [P3, P2] → G, P → 0/2 matches
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, None, Purple, Green));
    }

    // -------------------------------------------------------------------------
    // Three occupied slots — PPG_23 preload (P1=Purple, P2=Purple, P3=Green)
    //
    // Manual cost verification (starting from P1):
    //   GPP_21 expects [G, P, P]:
    //     [P3,P2,P1]: 3/3 matches, cost = 2+1+1 = 4  ← winner
    //     [P3,P1,P2]: 3/3 matches, cost = 2+2+1 = 5
    //   PGP_22 expects [P, G, P]:
    //     [P1,P3,P2]: 3/3 matches, cost = 0+2+1 = 3  ← winner
    //     [P2,P3,P1]: 3/3 matches, cost = 1+1+2 = 4
    //   PPG_23 expects [P, P, G]:
    //     [P1,P2,P3]: 3/3 matches, cost = 0+1+1 = 2  ← winner
    //     [P2,P1,P3]: 3/3 matches, cost = 1+1+2 = 4
    // -------------------------------------------------------------------------

    @Test
    public void testPPG23preload_obeliskGPP21_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3, SPIN_P2, SPIN_P1},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P1, Purple, Purple, Green));
    }

    @Test
    public void testPPG23preload_obeliskPGP22_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P3, SPIN_P2},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P1, Purple, Purple, Green));
    }

    @Test
    public void testPPG23preload_obeliskPPG23_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, Purple, Purple, Green));
    }

    // -------------------------------------------------------------------------
    // Three occupied slots — GPP_21 preload (P1=Green, P2=Purple, P3=Purple)
    //
    //   GPP_21 expects [G, P, P]:
    //     [P1,P2,P3]: 3/3 matches, cost from P1 = 0+1+1 = 2  ← winner
    //     [P1,P3,P2]: 3/3 matches, cost from P1 = 0+2+1 = 3
    // -------------------------------------------------------------------------

    @Test
    public void testGPP21preload_obeliskGPP21_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P1, Green, Purple, Purple));
    }

    // -------------------------------------------------------------------------
    // Movement tiebreaking — all same color, color-match score is equal for all
    // permutations; the winner is determined purely by move cost.
    //
    //   All Purple, obelisk PPG_23 expects [P, P, G]:
    //   Every permutation scores 2/3 matches (the two Purple slots always match
    //   the two Purple positions in the expected sequence; the Green never matches).
    //
    //   From P1:
    //     [P1,P2,P3]: cost = 0+1+1 = 2  ← minimum
    //   From P3:
    //     [P3,P2,P1]: cost = 0+1+1 = 2  ← minimum
    // -------------------------------------------------------------------------

    @Test
    public void testMoveCostTiebreak_allPurple_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, Purple, Purple, Purple));
    }

    @Test
    public void testMoveCostTiebreak_allPurple_fromP3() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3, SPIN_P2, SPIN_P1},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P3, Purple, Purple, Purple));
    }

    // -------------------------------------------------------------------------
    // Starting position affects the optimal order even when color matches tie
    // -------------------------------------------------------------------------

    @Test
    public void testStartingSlotIncluded_fromP3() {
        // PPG_23 preload, obelisk GPP_21.  From P3, the Green ball is already
        // at the current slot so the first move costs 0 instead of 2.
        //   [P3,P2,P1]: 3/3 matches, cost = 0+1+1 = 2  ← winner
        //   [P3,P1,P2]: 3/3 matches, cost = 0+2+1 = 3
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3, SPIN_P2, SPIN_P1},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P3, Purple, Purple, Green));
    }

    // -------------------------------------------------------------------------
    // Three occupied slots — PGP_22 preload (P1=Purple, P2=Green, P3=Purple)
    //
    // Manual cost verification (starting from P1):
    //   GPP_21 expects [G, P, P]:
    //     [P2,P1,P3]: 3/3 matches, cost = 1+1+2 = 4  ← winner (ties [P2,P3,P1] but comes first)
    //     [P2,P3,P1]: 3/3 matches, cost = 1+1+2 = 4
    //   PGP_22 expects [P, G, P]:
    //     [P1,P2,P3]: 3/3 matches, cost = 0+1+1 = 2  ← winner
    //     [P3,P2,P1]: 3/3 matches, cost = 2+1+1 = 4
    //   PPG_23 expects [P, P, G]:
    //     [P1,P3,P2]: 3/3 matches, cost = 0+2+1 = 3  ← winner
    //     [P3,P1,P2]: 3/3 matches, cost = 2+2+1 = 5
    // -------------------------------------------------------------------------

    @Test
    public void testPGP22preload_obeliskGPP21_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P1, SPIN_P3},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P1, Purple, Green, Purple));
    }

    @Test
    public void testPGP22preload_obeliskPGP22_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P1, Purple, Green, Purple));
    }

    @Test
    public void testPGP22preload_obeliskPPG23_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P3, SPIN_P2},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, Purple, Green, Purple));
    }

    // -------------------------------------------------------------------------
    // Three occupied slots — GPP_21 preload, remaining obelisk patterns from P1
    // (P1=Green, P2=Purple, P3=Purple)
    //
    //   PGP_22 expects [P, G, P]:
    //     [P2,P1,P3]: 3/3 matches, cost = 1+1+2 = 4  ← winner
    //     [P3,P1,P2]: 3/3 matches, cost = 2+2+1 = 5
    //   PPG_23 expects [P, P, G]:
    //     [P2,P3,P1]: 3/3 matches, cost = 1+1+2 = 4  ← winner (ties [P3,P2,P1] but comes first)
    //     [P3,P2,P1]: 3/3 matches, cost = 2+1+1 = 4
    // -------------------------------------------------------------------------

    @Test
    public void testGPP21preload_obeliskPGP22_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P1, SPIN_P3},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P1, Green, Purple, Purple));
    }

    @Test
    public void testGPP21preload_obeliskPPG23_fromP1() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P3, SPIN_P1},
                getObeliskShootOrder(BallOrder.PPG_23, SPIN_P1, Green, Purple, Purple));
    }

    // -------------------------------------------------------------------------
    // Starting from P2 changes the optimal order
    //
    // PPG_23 preload (P1=Purple, P2=Purple, P3=Green), obelisk PGP_22 [P,G,P]:
    //   From P1: [P1,P3,P2] — cost 0+2+1 = 3, score 6  ← wins from P1
    //   From P2: [P2,P3,P1] — cost 0+1+2 = 3, score 6  ← wins from P2
    //   The starting slot is different because beginning at P2 makes P2→P3→P1 cheapest.
    // -------------------------------------------------------------------------

    @Test
    public void testStartingP2_PPG23preload_obeliskPGP22() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P3, SPIN_P1},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P2, Purple, Purple, Green));
    }

    // -------------------------------------------------------------------------
    // Starting from P3 changes the optimal order
    //
    // PGP_22 preload (P1=Purple, P2=Green, P3=Purple), obelisk PGP_22 [P,G,P]:
    //   From P1: [P1,P2,P3] — cost 0+1+1 = 2, score 6  ← wins from P1
    //   From P3: [P3,P2,P1] — cost 0+1+1 = 2, score 6  ← wins from P3
    //   From P3 starting at the purple end makes P3→P2→P1 the lowest-cost path.
    // -------------------------------------------------------------------------

    @Test
    public void testStartingP3_PGP22preload_obeliskPGP22() {
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P3, SPIN_P2, SPIN_P1},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P3, Purple, Green, Purple));
    }

    // -------------------------------------------------------------------------
    // Two occupied slots — PGP_22 obelisk (previously untested with 2 balls)
    // -------------------------------------------------------------------------

    @Test
    public void testTwoSlots_PGP22obelisk_correctOrderFirst() {
        // P1=None, P2=Purple, P3=Green; obelisk PGP_22 expects [P, G, P]
        // [P2, P3] → P, G → 2/2 color matches, cost from P1: 1+1 = 2  ← winner
        // [P3, P2] → G, P → 0/2 matches
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P2, SPIN_P3},
                getObeliskShootOrder(BallOrder.PGP_22, SPIN_P1, None, Purple, Green));
    }

    @Test
    public void testTwoSlots_GPP21obelisk_skipMiddleEmpty() {
        // P1=Green, P2=None, P3=Purple; obelisk GPP_21 expects [G, P, P]
        // [P1, P3] → G, P → 2/2 color matches, cost from P1: 0+2 = 2  ← winner
        // [P3, P1] → P, G → 0/2 matches
        assertArrayEquals(new HardwareSwyftBot.SpindexerState[]{SPIN_P1, SPIN_P3},
                getObeliskShootOrder(BallOrder.GPP_21, SPIN_P1, Green, None, Purple));
    }
}
