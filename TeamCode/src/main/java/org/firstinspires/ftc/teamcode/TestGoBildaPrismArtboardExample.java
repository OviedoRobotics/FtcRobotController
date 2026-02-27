/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.GoBildaPrismDriver.Artboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareDrivers.Prism.GoBildaPrismDriver;


import java.util.concurrent.TimeUnit;

/*
 * This example shows how to recall previously created Artboards on the goBILDA Prism RGB Driver.
 * Recalling these animations is all we recommend that your main robot OpMode do. We prefer to
 * create these Artboards once and store them on the device, allowing for a short I²C write to
 * set the current Artboard.
 *
 * It also shows how to enable or disable the default boot animation on power up. When enabled,
 * as soon as the Prism gets power it will display the Artboard saved in Artboard slot 0. If you'd
 * instead like to wait until you explicitly set an Artboard, disable this setting. This setting is
 * saved and will only need to be set once.
 *
 */

@TeleOp(name="Prism Artboard Example", group="Linear OpMode")
//@Disabled

public class TestGoBildaPrismArtboardExample extends LinearOpMode {

    GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {

        /*
         * Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the driver's station.
         */
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addData("Device ID: ", prism.getDeviceID());
        telemetry.addData("Firmware Version: ", prism.getFirmwareVersionString());
        telemetry.addData("Hardware Version: ", prism.getHardwareVersionString());
        telemetry.addData("Power Cycle Count: ", prism.getPowerCycleCount());
        telemetry.addData("Run Time (Minutes): ", prism.getRunTime(TimeUnit.MINUTES));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.aWasPressed()){
                prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);
            } else if(gamepad1.bWasPressed()){
                prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1);
            } else if(gamepad1.yWasPressed()){
                prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2);
            } else if(gamepad1.xWasPressed()){
                prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3);
            }

            if(gamepad1.dpadLeftWasPressed()){prism.enableDefaultBootArtboard(false);}
            if(gamepad1.dpadRightWasPressed()){prism.enableDefaultBootArtboard(true);}

            telemetry.addLine("Press A to recall Artboard #0");
            telemetry.addLine("Press B to recall Artboard #1");
            telemetry.addLine("Press Y to recall Artboard #2");
            telemetry.addLine("Press X to recall Artboard #3");
            telemetry.addLine("");
            telemetry.addLine("By default the Prism will wait for an I²C signal to " +
                    " start showing an animation.The prism can enable a Default Boot animation" +
                    " which will play the animation stored at Artboard #0 automatically on power up.");
            telemetry.addLine("Press D-Pad Right to enable the default boot animation." +
                    " Press D-Pad Left to disable it.");
            telemetry.update();
            sleep(20);

        }
    }
}