package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleOp", group = "LinearOpMode")
public class teleOp extends LinearOpMode {
    Hardware robot = new Hardware();
    private DriveSpeed driveSpeed;
    private Hardware.DriveDirection driveDirection = Hardware.DriveDirection.FORWARD;
    private double deadzone;
    private double vertical, strafe, horizontal;
    private double maxSpeed;

    private enum DriveSpeed {
        FAST,
        SLOW,
        EXTRA_SLOW
    }

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        driveSpeed = DriveSpeed.FAST;
        vertical = 0;
        strafe = 0;
        horizontal = 0;
        deadzone = 0.08;
        maxSpeed = 0.8;
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("fl wheel ticks: ", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("fr wheel ticks: ", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("bl wheel ticks: ", robot.backLeftDrive.getCurrentPosition());
            telemetry.addData("br wheel ticks: ", robot.backRightDrive.getCurrentPosition());
            telemetry.update();

            robot.mecanumDrive(vertical, strafe, horizontal, driveDirection, maxSpeed);

            if(Math.abs(gamepad1.left_stick_y) > deadzone) {
                vertical = -gamepad1.left_stick_y;
            } else {
                vertical = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) > deadzone) {
                strafe = gamepad1.left_stick_x;
            } else {
                strafe = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > deadzone) {
                if(driveDirection == Hardware.DriveDirection.FORWARD) {
                    horizontal = -gamepad1.right_stick_x;
                } else {
                    horizontal = gamepad1.right_stick_x;
                }
            } else {
                horizontal = 0;
            }


            if (gamepad1.left_trigger != 0) {
                driveSpeed = DriveSpeed.EXTRA_SLOW;
                maxSpeed = 0.2;
            } else if (gamepad1.right_trigger != 0) {
                driveSpeed = DriveSpeed.SLOW;
                maxSpeed = 0.4;
            } else {
                driveSpeed = DriveSpeed.FAST;
                maxSpeed = 1;
            }


            if (gamepad1.y){
                driveDirection = Hardware.DriveDirection.FORWARD;
            } else if(gamepad1.x) {
                driveDirection = Hardware.DriveDirection.REVERSE;
            }

            if(gamepad2.left_stick_y != 0) {
                robot.intake.setPower(gamepad2.left_stick_y);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad2.right_stick_y != 0) {
                robot.testServo.setPosition(0);
            } else {
                robot.testServo.setPosition(0.5);
            }

        }
    }
}