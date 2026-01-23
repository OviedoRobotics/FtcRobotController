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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * goBILDA Laser Distance Sensor Example (Analog Mode)
 *
 * Reads the analog output (0–3.3V) of the Dual-Mode Laser Distance Sensor
 * and converts it linearly to distance (0–1000 mm).
 *
 * 0.0V →   0 mm
 * 3.3V → 1000 mm
 *
 * Wiring/Config:
 * - Connect the sensor’s analog signal to a Hub Analog port.
 * - Name the device "laserAnalog" in Robot Configuration.
 *
 * Display:
 * - Driver Station telemetry shows voltage and distance (mm).
 */
@TeleOp(name = "TestLaserAnalog", group = "Test")
@Disabled
public class TestLaserAnalog extends LinearOpMode {

    private AnalogInput laserAnalog;

    // Sensor scale: 3.3V corresponds to 1000 mm
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    @Override
    public void runOpMode() {
        // Map the analog device from the hardware configuration
        laserAnalog = hardwareMap.get(AnalogInput.class, "leftPresence");  // analog 2 (2-3)

        // Wait for PLAY
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read sensor voltage (0.0–3.3V)
            double volts = laserAnalog.getVoltage();

            // Convert voltage to distance in millimeters (linear mapping)
            double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

            // Telemetry
            telemetry.addData("Voltage (V)", "%.3f", volts);
            telemetry.addData("Distance (mm)", "%.1f", distanceMM);
            telemetry.update();
        }
    }
} // TestLaserAnalog