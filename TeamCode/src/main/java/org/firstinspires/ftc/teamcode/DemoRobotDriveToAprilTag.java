/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="Drive To AprilTag", group = "Demo")
//@Disabled
public class DemoRobotDriveToAprilTag extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 3.0; //  this is how close the camera should get to the target (inches)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private boolean rearFacingWebcam = false;

    // CENTERSTAGE AprilTag assignments:
    //  - Blue Alliance LEFT   Backdrop  = 1        - Red Alliance LEFT   Backdrop  = 4
    //  - Blue Alliance CENTER Backdrop  = 2        - Red Alliance CENTER Backdrop  = 5
    //  - Blue Alliance RIGHT  Backdrop  = 3        - Red Alliance RIGHT  Backdrop  = 6
    //  - Blue Alliance 5-stack 2"/50mm  = 9        - Red Alliance 5-stack 2"/50mm  = 8
    //  - Blue Alliance 5-stack 5"/127mm = 10       - Red Alliance 5-stack 5"/127mm = 7
    private static final int DESIRED_TAG_ID = 2;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectionData = null;     // Used to hold the data for a detected AprilTag

    double  aprilDriveFwd    = 0.0;      // Desired forward power/speed (-1 to +1)
    double  aprilDriveStrafe = 0.0;      // Desired strafe power/speed (-1 to +1)
    double  aprilDriveTurn   = 0.0;      // Desired turning power/speed (-1 to +1)

    @Override public void runOpMode()
    {
        boolean targetVisible    = false;    // Set to true when an AprilTag target is detected

        boolean hasRumbled = false;
        Gamepad.RumbleEffect meowRumbleEffect1;

        meowRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble left/right motors 100% for 500 mSec
                .build();

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "RearLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            // Does AprilTag processor detect our desired tag? (-1 for ANY tag)
            // (when true, detectionData holds the results)
            targetVisible = processAprilTagDetections( DESIRED_TAG_ID );
            
            // Tell the driver what we see, and what to do.
            if (targetVisible) {
                if(!hasRumbled) {
                    gamepad1.runRumbleEffect(meowRumbleEffect1);
                    hasRumbled = true;
                }
                telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", detectionData.id, detectionData.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", detectionData.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", detectionData.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", detectionData.ftcPose.yaw);
            } else {
                hasRumbled = false;
                telemetry.addData(">","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetVisible) {

                // Convert AprilTag pose data (range,yaw,bearing) in detectionData into
                // motor powers (aprilDriveFwd, aprilDriveStrafe, aprilDriveTurn)
                processAprilTagCorrections( DESIRED_DISTANCE );
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilDriveFwd, aprilDriveStrafe, aprilDriveTurn);
            } else {
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                aprilDriveFwd    = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                aprilDriveStrafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                aprilDriveTurn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                if( Math.abs(aprilDriveFwd)    < 0.03 ) aprilDriveFwd    = 0.0;
                if( Math.abs(aprilDriveStrafe) < 0.03 ) aprilDriveStrafe = 0.0;
                if( Math.abs(aprilDriveTurn)   < 0.05 ) aprilDriveTurn   = 0.0;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", aprilDriveFwd, aprilDriveStrafe, aprilDriveTurn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            if( rearFacingWebcam ) {
                moveRobot(-aprilDriveFwd, -aprilDriveStrafe, -aprilDriveTurn);
            } else {
                moveRobot(aprilDriveFwd, aprilDriveStrafe, aprilDriveTurn);
            }
            sleep(10);
        }
    } //  runOpMode()
    
    /*---------------------------------------------------------------------------------*/
     boolean processAprilTagDetections( int desiredAprilTagID ) {
        boolean successfulDetection = false;

        // discard any prior detection data
        detectionData  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredAprilTagID < 0) || (detection.id == desiredAprilTagID)) {
                    // Yes, we want to use this tag.
                    successfulDetection = true;
                    detectionData = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        } // for()

     return successfulDetection;

     } // processAprilTagDetections

    /*------------------------------------------------------------------------------------------*/
    /* Where do we need to move to achieve the desired offset distance (zero yaw/heading error) */
     void processAprilTagCorrections( double desiredDistance ) {
        
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is applied
        //  to the drive motors to correct the error.  Smaller gains provide smoother control; larger produce more aggressive response.
        //    Drive = Error * Gain    
        final double SPEED_GAIN      =  0.0100;  //  Forward Speed Control
        final double STRAFE_GAIN     =  0.00005; //  Strafe Speed Control
        final double TURN_GAIN       =  0.00005; //  Turn Control

        final double MAX_AUTO_SPEED  = 0.75;     //  Clip the approach speed to this max value
        final double MAX_AUTO_STRAFE = 0.50;     //  Clip the approach speed to this max value
        final double MAX_AUTO_TURN   = 0.30;     //  Clip the turn speed to this max value
         
        final double minPwrDrive     = 0.08;     // minimum power needed to drive robot forward
        final double minPwrStrafe    = 0.26;
        final double minPwrTurn      = 0.17;

        double  errorPwrDrive, errorPwrStrafe, errorPwrTurn;

        double  errorInchesRange    = (detectionData.ftcPose.range - desiredDistance);
        double  errorDegreesYaw     = -detectionData.ftcPose.yaw;
        double  errorDegreesHeading = detectionData.ftcPose.bearing;
        telemetry.addData("Error","Drive %.2f in, Strafe %.2f deg, Turn %.2f deg", errorInchesRange, errorDegreesYaw, errorDegreesHeading);

        // Convert the current range error in INCHES to MOTOR POWERS
        if( Math.abs(errorInchesRange) < 0.3 ) {    // Within 0.3" is considered DONE
            errorPwrDrive = 0.0;
        } else if( errorInchesRange > 0 ) {
            errorPwrDrive  = minPwrDrive + (errorInchesRange * SPEED_GAIN);
        }
        else { // errorInchesRange < 0
            errorPwrDrive  = -minPwrDrive + (errorInchesRange * SPEED_GAIN);
        }

        // Only correct for YAW/STRAFE when it's really bad (more than 5deg)
        if( Math.abs(errorDegreesYaw) < 5.0 ) {  // degrees
            errorPwrStrafe = 0.0;
        } else if( errorDegreesYaw > 0 ) {
            errorPwrStrafe  = minPwrStrafe + (errorDegreesYaw * STRAFE_GAIN);
        }
        else { // errorDegreesYaw < 0
            errorPwrStrafe = -minPwrStrafe + (errorDegreesYaw * STRAFE_GAIN);
        }

        // Convert the current heading error in DEGREES to MOTOR POWER
        if( Math.abs(errorDegreesHeading) < 2.0 ) {  // degrees
            errorPwrTurn = 0.0;
        } else if( errorDegreesHeading > 0 ) {
            errorPwrTurn  = minPwrTurn + (errorDegreesHeading * TURN_GAIN);
        }
        else { // errorDegreesHeading < 0
            errorPwrTurn = -minPwrTurn + (errorDegreesHeading * TURN_GAIN);
        }
        
        // Correct for any large STRAFE error (to better center on the AprilTag)
        // Do this with no FORWARD/TURN correction that would affect our minPower
        if( Math.abs(errorPwrStrafe) >= minPwrStrafe ) {
            aprilDriveFwd    = 0.0;
            aprilDriveTurn   = 0.0;
            aprilDriveStrafe = Range.clip( errorPwrStrafe, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        } else {
            aprilDriveFwd    = Range.clip( errorPwrDrive, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            aprilDriveTurn   = Range.clip( errorPwrTurn,  -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            aprilDriveStrafe = 0.0;
        }
         
     } // processAprilTagCorrections

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     *
     *       FORWARD-FACING CAMERA:             REAR-FACING CAMERA:
     *
     *    (left front)  == (right front)      (right back) === (left back)
     *     ||                        ||        ||                       ||
     *    (left back)  === (right back)       (right front) == (left front)
     */
    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        // Calculate wheel powers.
        if( rearFacingWebcam ) {
            rightBackPower  = x -y -yaw;
            leftBackPower   = x +y +yaw;
            rightFrontPower = x +y -yaw;
            leftFrontPower  = x -y +yaw;

        } else {
            leftFrontPower  =  x -y -yaw;
            rightFrontPower =  x +y +yaw;
            leftBackPower   =  x +y -yaw;
            rightBackPower  =  x -y +yaw;
        }

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // If you do not manually specify calibration parameters, the SDK will
                // to load a predefined calibration for your camera.
              //  ===== CAMERA CALIBRATION for 150deg webcam ===
              //.setLensIntrinsics(332.309,332.309,341.008,243.109)
              //  ===== CAMERA CALIBRATION for Arducam B0197 webcam ===
              //.setLensIntrinsics(1566.16,1566.16,1002.58,539.862)
              //  ===== CAMERA CALIBRATION for Arducam B0385 webcam ===
                .setLensIntrinsics(904.214,904.214,696.3,362.796)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280,800))
                .addProcessor(aprilTag)
                .build();
        rearFacingWebcam = true;
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
