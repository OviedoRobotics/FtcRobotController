package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.calcAngleDegToTarget;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.calcShootAngleDeg;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class TargetPositionTest {

    @Test
    void testOriginStraight() {
        assertEquals(43, calcShootAngleDeg(Alliance.BLUE, 0.0, 0.0, 0.0), 1);
        assertEquals(-43, calcShootAngleDeg(Alliance.RED, 0.0, 0.0, 0.0), 1);
    }

    @Test
    void testStraightFromOpponentBase() {
        assertEquals(12.4, calcShootAngleDeg(Alliance.BLUE, -40, 34, 0.0), 1);
        assertEquals(0.0, calcShootAngleDeg(Alliance.BLUE, -40, 34, 12.4), 1);
        assertEquals(-12.4, calcShootAngleDeg(Alliance.RED, -40, -34, 0.0), 1);
        assertEquals(0.0, calcShootAngleDeg(Alliance.RED, -40, -34, -12.4), 1);
    }

    @Test
    void testStraightY() {
        assertEquals(0.0, calcShootAngleDeg(Alliance.BLUE, -20, 55.64, 0.0), 0.01);
        assertEquals(0.0, calcShootAngleDeg(Alliance.RED, -20, -55.64, 0.0), 0.01);
    }

    @Test
    void testStraightX() {
        assertEquals(0.0, calcShootAngleDeg(Alliance.BLUE, 58.37, 0, 90), 0.01);
        assertEquals(0.0, calcShootAngleDeg(Alliance.RED, 58.37, 0, -90), 0.01);
    }

    @Test
    void testTargetX1() {
        assertEquals(0.0, calcAngleDegToTarget(1, 0, 0, 0, 0), 0.01);
        assertEquals(-10.0, calcAngleDegToTarget(1, 0, 0, 0, 10), 0.01);
        assertEquals(10.0, calcAngleDegToTarget(1, 0, 0, 0, -10), 0.01);
    }

    @Test
    void testTargetX1Y1() {
        assertEquals(45.0, calcAngleDegToTarget(1, 1, 0, 0, 0), 0.01);
        assertEquals(0.0, calcAngleDegToTarget(1, 1, 0, 0, 45), 0.01);
        assertEquals(90.0, calcAngleDegToTarget(1, 1, 0, 0, -45), 0.01);
    }

    @Test
    void testTargetY1() {
        assertEquals(90.0, calcAngleDegToTarget(0, 1, 0, 0, 0), 0.01);
    }

    @Test
    void testTarget_X1() {
        assertEquals(180.0, calcAngleDegToTarget(-1, 0, 0, 0, 0), 0.01);
    }

    @Test
    void testTarget_Y1() {
        assertEquals(-90.0, calcAngleDegToTarget(0, -1, 0, 0, 0), 0.01);
    }

    @Test
    void testTarget_X1Y1() {
        assertEquals(-135.0, calcAngleDegToTarget(-1, -1, 0, 0, 0), 0.01);
    }
}
