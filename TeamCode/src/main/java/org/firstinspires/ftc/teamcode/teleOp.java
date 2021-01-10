package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleOp", group = "LinearOpMode")
public class teleOp extends LinearOpMode {
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftStrafe = -gamepad1.left_trigger;
            double rightStrafe = -gamepad1.right_trigger;
            double backAndForth = -gamepad1.left_stick_y;
            double sideMovement = -gamepad1.left_stick_x;

            if (gamepad1.left_stick_y != 0) {
                robot.frontLeftDrive.setPower(backAndForth);
                robot.backLeftDrive.setPower(backAndForth);
                robot.frontRightDrive.setPower(backAndForth);
                robot.backRightDrive.setPower(backAndForth);
            } else if (gamepad1.left_stick_x != 0) {
                robot.frontLeftDrive.setPower(-sideMovement);
                robot.backLeftDrive.setPower(sideMovement);
                robot.frontRightDrive.setPower(sideMovement);
                robot.backRightDrive.setPower(-sideMovement);
            } else if (gamepad1.left_trigger != 0) {
                robot.frontLeftDrive.setPower(-leftStrafe);
                robot.backLeftDrive.setPower(leftStrafe);
                robot.frontRightDrive.setPower(leftStrafe);
                robot.backRightDrive.setPower(-leftStrafe);
            } else if (gamepad1.right_trigger != 0) {
                robot.frontLeftDrive.setPower(rightStrafe);
                robot.backLeftDrive.setPower(-rightStrafe);
                robot.frontRightDrive.setPower(-rightStrafe);
                robot.backRightDrive.setPower(rightStrafe);
            } else if (gamepad1.right_stick_y != 0) {
                robot.frontLeftDrive.setPower(gamepad1.right_stick_y);
                robot.backLeftDrive.setPower(gamepad1.right_stick_y);
                robot.frontRightDrive.setPower(-gamepad1.right_stick_y);
                robot.backRightDrive.setPower(-gamepad1.right_stick_y);
            } else {
                robot.frontLeftDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                robot.frontRightDrive.setPower(0);
                robot.backRightDrive.setPower(0);
            }
        }
    }
}
