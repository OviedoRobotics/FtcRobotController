package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.Color;
import org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.PrismAnimations;

/*
 * Demonstrates integrating the GoBILDA Prism LED driver into the robot's visual status display.
 *
 * LED Strip Layout (6 LEDs):
 *   Left 3  (indices 0–2) — Spinventory: one LED per spindexer ball slot
 *     LED 0 → Left   slot
 *     LED 1 → Center slot
 *     LED 2 → Right  slot
 *   Right 3 (indices 3–5) — Shoot status: all three LEDs share the same color
 *
 * Ball colors (spinventory):
 *   Ball.None   → dim white  (10% brightness)
 *   Ball.Purple → purple     (70% brightness)
 *   Ball.Green  → green      (70% brightness)
 *
 * Shoot-status colors:
 *   Flywheel speed AND turret angle both ready → blue  (100%)
 *   Either not ready                           → orange (70%)
 *
 * Gamepad controls (simulation — no robot hardware required):
 *   Square   (□) → cycle Left   slot:  None → Purple → Green → None
 *   Triangle (△) → cycle Center slot
 *   Circle   (○) → cycle Right  slot
 *   Cross    (✕) → toggle flywheel speed ready
 *   L1           → toggle turret angle ready
 *   Dpad Down    → clear all animations
 */
@TeleOp(name = "Prism Robot Integration Test", group = "Test")
//@Disabled
public class TestGoBildaPrismRobotIntegration extends LinearOpMode {

    // Mirrors HardwareSwyftBot.Ball
    enum Ball { None, Purple, Green }

    // Simulated spinventory, indexed by LED position:
    //   slot[0] = Left   → LED 0
    //   slot[1] = Center → LED 1
    //   slot[2] = Right  → LED 2
    Ball[] spinventory = { Ball.None, Ball.None, Ball.None };

    // Simulated shoot-status flags (mirrors Teleop local variables)
    boolean speedReady = false;
    boolean angleReady = false;

    // One Solid animation per spinventory LED (initial state: dim white = empty slot)
    // Plus one Solid spanning LEDs 3–5 for shoot status (initial state: orange = not ready)
    PrismAnimations.Solid ledLeft   = new PrismAnimations.Solid(Color.WHITE,  10,  0, 0);
    PrismAnimations.Solid ledCenter = new PrismAnimations.Solid(Color.WHITE,  10,  1, 1);
    PrismAnimations.Solid ledRight  = new PrismAnimations.Solid(Color.WHITE,  10,  2, 2);
    PrismAnimations.Solid ledShoot  = new PrismAnimations.Solid(Color.ORANGE, 70,  3, 5);

    GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // Establish all four animations before the match begins
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, ledLeft);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, ledCenter);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, ledRight);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, ledShoot);

        telemetry.addLine("Prism initialized — waiting for START");
        telemetry.addData("Device ID",        prism.getDeviceID());
        telemetry.addData("Firmware",         prism.getFirmwareVersionString());
        telemetry.addData("Number of LEDs",   prism.getNumberOfLEDs());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Spinventory simulation ────────────────────────────────────────────
            if (gamepad1.squareWasPressed()) {      // □ Left slot → LED 0
                spinventory[0] = nextBall(spinventory[0]);
                applySpinventoryColor(ledLeft, spinventory[0]);
                prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            }
            if (gamepad1.triangleWasPressed()) {    // △ Center slot → LED 1
                spinventory[1] = nextBall(spinventory[1]);
                applySpinventoryColor(ledCenter, spinventory[1]);
                prism.updateAnimationFromIndex(LayerHeight.LAYER_1);
            }
            if (gamepad1.circleWasPressed()) {      // ○ Right slot → LED 2
                spinventory[2] = nextBall(spinventory[2]);
                applySpinventoryColor(ledRight, spinventory[2]);
                prism.updateAnimationFromIndex(LayerHeight.LAYER_2);
            }

            // ── Shoot-status simulation ───────────────────────────────────────────
            if (gamepad1.crossWasPressed()) {       // ✕ Toggle flywheel speed
                speedReady = !speedReady;
                applyShootColor();
            }
            if (gamepad1.leftBumperWasPressed()) {  // L1 Toggle turret angle
                angleReady = !angleReady;
                applyShootColor();
            }

            // ── Utility ───────────────────────────────────────────────────────────
            if (gamepad1.dpadDownWasPressed()) {
                prism.clearAllAnimations();
            }

            // ── Telemetry ─────────────────────────────────────────────────────────
            telemetry.addLine("─── Spinventory  (Left 3 LEDs) ───");
            telemetry.addData("LED 0  Left   [□] ", spinventory[0]);
            telemetry.addData("LED 1  Center [△] ", spinventory[1]);
            telemetry.addData("LED 2  Right  [○] ", spinventory[2]);
            telemetry.addLine("─── Shoot Status (Right 3 LEDs) ───");
            telemetry.addData("Speed ready  [✕] ", speedReady);
            telemetry.addData("Angle ready  [L1]", angleReady);
            telemetry.addData("All ready        ", speedReady && angleReady);
            telemetry.addLine();
            telemetry.addLine("D-Pad Down = clear all animations");
            telemetry.update();

            sleep(50);
        }
    }

    /** Cycle:  None → Purple → Green → None */
    private Ball nextBall(Ball current) {
        switch (current) {
            case None:   return Ball.Purple;
            case Purple: return Ball.Green;
            default:     return Ball.None;
        }
    }

    /** Set the Solid animation's color + brightness to match the ball state. */
    private void applySpinventoryColor(PrismAnimations.Solid led, Ball ball) {
        switch (ball) {
            case None:
                led.setPrimaryColor(Color.WHITE);
                led.setBrightness(10);   // dim white = empty slot
                break;
            case Purple:
                led.setPrimaryColor(Color.PURPLE);
                led.setBrightness(70);
                break;
            case Green:
                led.setPrimaryColor(Color.GREEN);
                led.setBrightness(70);
                break;
        }
    }

    /** Update the three shoot-status LEDs (3–5) based on the current speed/angle flags. */
    private void applyShootColor() {
        if (speedReady && angleReady) {
            ledShoot.setPrimaryColor(Color.BLUE);
            ledShoot.setBrightness(100);
        } else {
            ledShoot.setPrimaryColor(Color.ORANGE);
            ledShoot.setBrightness(70);
        }
        prism.updateAnimationFromIndex(LayerHeight.LAYER_3);
    }
}
