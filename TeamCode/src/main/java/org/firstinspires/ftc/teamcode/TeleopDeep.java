package org.firstinspires.ftc.teamcode;


import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TeleopDeep", group = "DeepBot")
public class TeleopDeep extends LinearOpMode {
    HardwareDeepBot robot = new HardwareDeepBot();

    static final double     DRIVE_SPEED             = 0.9;
    static final double     SWING_ARM_POW           = 0.3;
    static final double     VIPER_POW               = 0.7;
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad2_triangle_last, gamepad2_triangle_now = false;
    boolean gamepad2_square_last, gamepad2_square_now = false;
    boolean gamepad2_dpad_up_last, gamepad2_dpad_up_now = false;
    boolean gamepad2_dpad_down_last, gamepad2_dpad_down_now = false;
    boolean gamepad2_left_trigger_last, gamepad2_left_trigger_now = false;
    double gamepad2_left_trigger;
    boolean gamepad2_right_trigger_last, gamepad2_right_trigger_now = false;
    double gamepad2_right_trigger;
    boolean gamepad2_left_bumper_last, gamepad2_left_bumper_now = false;
    boolean gamepad2_right_bumper_last, gamepad2_right_bumper_now = false;
    double gamepad2_right_stick_y;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initializing... please wait");
        telemetry.update();

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run)");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            captureGamepad1Buttons();
            captureGamepad2Buttons();

            processSampleGrab();
            processSwingArm();
            processViperSlides();
            if(!processDpads()) {
                processJoysticks();
            }
            telemetry.update();
        }
        robot.setMotorPowers(0);
    }


    void processSampleGrab(){
        if(gamepad2_triangle_now && !gamepad2_triangle_last){
            robot.clawServo1.setPosition(robot.CLAW1_SERVO_GRAB);
        }
        else if(gamepad2_square_now && !gamepad2_square_last){
            robot.clawServo1.setPosition(robot.CLAW1_SERVO_EXPAND);
        }
    }

    void processSwingArm(){
        boolean liftingUp = false;
        if(gamepad2_dpad_up_now && !gamepad2_dpad_up_last){
            robot.startArmPivotAutomatic(robot.ARM_HIGH);
        }
        else if(gamepad2_dpad_down_now && !gamepad2_dpad_down_last){
            robot.startArmPivotAutomatic(robot.ARM_GROUND);
        }
        else if(Math.abs(gamepad2_right_stick_y) > 0.05){
            liftingUp = (gamepad2_right_stick_y > 0)? true : false;
            //double vipMotorPow = (robot.calculateArmAngle(robot.swingArm.getCurrentPosition()
              //      - robot.ARM_PARALLEL) < 90)? VIPER_POW : -VIPER_POW;
            double armPow = (gamepad2_right_stick_y > 0)? SWING_ARM_POW : -SWING_ARM_POW;
            robot.swingArm.setPower(SWING_ARM_POW);
            robot.startArmPivotManual(armPow, VIPER_POW, liftingUp);
        }
    }
    /*void processSwingArm(){
        if(gamepad2_dpad_up_now){
            robot.swingArm.setPower(SWING_ARM_POW);
        }
        else if(gamepad2_dpad_down_now){
            robot.swingArm.setPower(-SWING_ARM_POW);
        }
        else{
            robot.swingArm.setPower(0.0);
        }
    }*/

    void processViperSlides(){
        if(gamepad2_left_trigger_now) {
            robot.viperMotor.setPower(VIPER_POW);
        }
        else if(gamepad2_right_trigger_now) {
            robot.viperMotor.setPower(-VIPER_POW);
        }
        else{
            robot.viperMotor.setPower(0.0);
        }
    }

    boolean processDpads() {
        if( gamepad1.dpad_up ) {
            telemetry.addData("Dpad","FORWARD");
            robot.setMotorPowers(DRIVE_SPEED);
        }
        else if( gamepad1.dpad_down ) {
            telemetry.addData("Dpad","BACKWARD");
            robot.setMotorPowers(-DRIVE_SPEED);
        }
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","LEFT");
            robot.setMotorPowers(-DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, -DRIVE_SPEED);
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","RIGHT");
            robot.setMotorPowers(DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED, DRIVE_SPEED);
        }
        else{
            robot.setMotorPowers(0);
            return false;
        }
        return true;
    }

    void processJoysticks(){
        double drivePowerY = -min(DRIVE_SPEED, gamepad1.left_stick_y);
        double drivePowerX = min(DRIVE_SPEED, gamepad1.left_stick_x);
        double turnPower   = gamepad1.right_stick_x;
        double motorPower = drivePowerY - drivePowerX - turnPower;
        robot.setMotorPowers(drivePowerY + drivePowerX + turnPower,
                drivePowerY - drivePowerX - turnPower,
                drivePowerY - drivePowerX + turnPower,
                drivePowerY + drivePowerX - turnPower);
    }

    void captureGamepad1Buttons(){
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;

    }

    void captureGamepad2Buttons(){
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now; gamepad2_dpad_up_now = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now; gamepad2_dpad_down_now = gamepad2.dpad_down;
        gamepad2_left_bumper_last  = gamepad2_left_bumper_now; gamepad2_left_bumper_now = gamepad2.left_bumper;
        gamepad2_right_bumper_last  = gamepad2_right_bumper_now; gamepad2_right_bumper_now = gamepad2.right_bumper;
        gamepad2_left_trigger    = gamepad2.left_trigger * 1.00;
        gamepad2_right_trigger   = gamepad2.right_trigger * 1.00;
        gamepad2_left_trigger_last = gamepad2_left_trigger_now;
        gamepad2_left_trigger_now = (gamepad2_left_trigger > 0.05)?  true : false;
        gamepad2_right_trigger_last = gamepad2_right_trigger_now;
        gamepad2_right_trigger_now = (gamepad2_right_trigger > 0.05)?  true : false;
        gamepad2_right_stick_y = gamepad2.right_stick_y;
    }

}
