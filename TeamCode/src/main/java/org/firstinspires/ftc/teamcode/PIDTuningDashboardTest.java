package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "PID Tuner", group = "7592")
public class PIDTuningDashboardTest extends LinearOpMode {
    HardwareMinibot robot = new HardwareMinibot();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private PIDController pid;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, true);

        // Initialize dashboard, controller, and dashboard + driver station telemetry (MultipleTelemetry)
        // 
        pid = new PIDController(PIDConstantsConfig.kP, PIDConstantsConfig.kI, PIDConstantsConfig.kD);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Wait for User to select what to graph by giving dummy data
        while (!isStarted() && !isStopRequested()) {
            double currentPosition = robot.headingIMU();
            telemetry.addData("Target Position", PIDConstantsConfig.targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Motor Power", 0.0);
            telemetry.update();
        }

        while (opModeIsActive()) {
            double currentPosition = robot.angleWrapper(robot.headingIMU());
            // Set motor turn power from pid controller
            double motorPower = pid.update(PIDConstantsConfig.targetPosition, currentPosition);
            robot.driveTrainTurn(motorPower);
            //Display Telemetry
            telemetry.addData("Target Position", PIDConstantsConfig.targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

        }
    }
}