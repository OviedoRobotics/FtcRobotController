package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="Test Shooter PIDF Control", group="Test")
//@Disabled
public class TestShooterPIDF extends OpMode {

    // Hardware
    public DcMotorEx shooterMotor1 = null;
    public DcMotorEx shooterMotor2 = null;
    
    public double highVelocity = 1320; // power = 0.55 (varies 1300-1340)
    public double lowVelocity = 900;
    public double curTargetVelocity = highVelocity;
    double P = 0; // 250 Proportional (multiplier for the current velocity "error")
    double F = 0; // 16  Feedforward (minimum power to keep motor running at target velocity)
    
    double [] stepSizes = { 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001 };
    int stepIndex = 1;

    @Override
    public void init() {

        // Initialize hardware
        shooterMotor1  = hardwareMap.get(DcMotorEx.class,"ShooterMotor1");
        shooterMotor2  = hardwareMap.get(DcMotorEx.class,"ShooterMotor2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor1.setPower( 0.0 );
        shooterMotor2.setPower( 0.0 );
        shooterMotor1.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients( P, 0, 0, F );
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        telemetry.addLine("Init complete");
    } // init
    
    @Override
    public void loop() {
        // get all our gamepad commands
        if( gamepad1.crossWasPressed() ) {
            curTargetVelocity = (curTargetVelocity == highVelocity)? lowVelocity : highVelocity;
        }
        
        if( gamepad1.circleWasPressed() ) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        
        if( gamepad1.dpadRightWasPressed() ) {
            F += stepSizes[stepIndex];
        }
        else if( gamepad1.dpadLeftWasPressed() ) {
            F -= stepSizes[stepIndex];
        }
        
        if( gamepad1.dpadUpWasPressed() ) {
            P += stepSizes[stepIndex];
        }
        else if( gamepad1.dpadDownWasPressed() ) {
            P -= stepSizes[stepIndex];
        }
        
        // set new PIDF coefficients
        PIDFCoefficients shooterPIDF = new PIDFCoefficients( P, 0, 0, F ); 
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);        
        
        // set target velocity
        shooterMotor1.setVelocity( curTargetVelocity );
        shooterMotor2.setVelocity( curTargetVelocity );
        
        double curVelocity1 = shooterMotor1.getVelocity();
        double curVelocity2 = shooterMotor2.getVelocity();
        double curVelocity = (curVelocity1 + curVelocity2)/2;
        
        double error = curVelocity - curTargetVelocity;

        // PROCESS:
        // 1) Start with P=0, F=0 and increase F until you achieve the midrange velocity
        //    you plan to use with the shooter (between MIN and MAX shooting distance).
        //    If you increase F too high, you'll get a positive error (above the target velocity)
        // 2) With F value established, tune the P value to where transitions between LOW and HIGH
        //    velocity targets occur quickly (0.1 sec versus 3 seconds).
        // Due to high mass of the dual-wheel steel plate flywheel, and low torque of the 6000 rpm
        // motor, expect P values in excess of 250.
        telemetry.addData("Target Velocity", "%.2f (X)", curTargetVelocity );
        telemetry.addData("Current Velocity","%.2f", curVelocity );
        telemetry.addData("Error", "%.2f", error );
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P", "%.4f (dpad U/D)", P );
        telemetry.addData("Tuning F", "%.4f (dpad L/R)", F );
        telemetry.addData("Step Size", "%.4f (O button)", stepSizes[stepIndex]);
     } // loop
    
} // TestShooterPIDF
