/* FTC Team 7572 - Version 1.0 (11/05/2025) */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp for BLUE alliance
 */
@TeleOp(name="Teleop-Blue", group="7592")
//@Disabled
public class TeleopBlue extends Teleop {

    @Override
    public void setAllianceSpecificBehavior() {
        // DECODE has different AprilTags for Red vs. Blue.
        blueAlliance = true;
        farAlliance  = true;

        // DECODE AprilTag assignments (tag family 36h11)
        aprilTagGoal = 20;  // Blue Alliance Goal
        
        // Note: common OBELISK april tags for both RED & BLUE alliance
        //  21 = GPP (green purple purple)
        //  22 = PGP (purple green purple)
        //  23 = PPG (purple purple green)
        
        
    }
} // TeleopBlue
