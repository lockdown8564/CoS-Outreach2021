package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "teleOp", group = "LinearOpMode")
public class teleOp extends LinearOpMode {
    Hardware robot = new Hardware();
    private DriveSpeed driveSpeed;
    private Hardware.DriveDirection driveDirection = Hardware.DriveDirection.FORWARD;
    private double deadzone;
    private double vertical, strafe, turn;
    private double maxSpeed;
    private double wobbleServoAdjust;
    private int newWobbleArmTargetPos;
    private double flywheel1Adjust = 1;
    private double flywheel2Adjust = -1;


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
        turn = 0;
        deadzone = 0.08;
        maxSpeed = 0.8;
        robot.init(hardwareMap);
        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        robot.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        robot.shooterServo.setPosition(1);
        runtime.reset();

        while (opModeIsActive()) {

            robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("fl wheel pos: ", robot.frontLeftDrive.getCurrentPosition());
            telemetry.addData("fr wheel pos: ", robot.frontRightDrive.getCurrentPosition());
            telemetry.addData("bl wheel pos: ", robot.backLeftDrive.getCurrentPosition());
            telemetry.addData("br wheel pos: ", robot.backRightDrive.getCurrentPosition());
            telemetry.addData("robot shooter power: ", flywheel1Adjust);
            telemetry.addData("robot flywheel power: ", robot.flywheel.getPower());
            telemetry.addData("wobble arm ticks: ", robot.wobbleArm.getCurrentPosition());
            telemetry.update();

            robot.mecanumDrive(vertical, strafe, turn, driveDirection, maxSpeed);

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
                    turn = gamepad1.right_stick_x;
                } else {
                    turn = -gamepad1.right_stick_x;
                }
            } else {
                turn = 0;
            }

//          sets drive speed of robot
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

//          sets drive motors to either forward or reverse mode
            if (gamepad1.y){
                driveDirection = Hardware.DriveDirection.FORWARD;
            } else if(gamepad1.x) {
                driveDirection = Hardware.DriveDirection.REVERSE;
            }

//          sets power of intake
            if(gamepad2.right_bumper) {
                robot.intake.setPower(1);
            } else if(gamepad2.left_bumper) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }

//          sets power of flywheel motors & servo for shooter (robot ring shooter)
            if(gamepad2.a) {
                robot.shooterServo.setPosition(0.3);
            } else {
                robot.shooterServo.setPosition(0.85);
            }

            if(gamepad2.b) {
                robot.flywheel.setPower(flywheel1Adjust);
                robot.flywheel2.setPower(flywheel2Adjust);
            } else if (gamepad2.right_trigger != 0) {
                robot.flywheel.setPower(0.74);
                robot.flywheel2.setPower(-0.74);
            } else {
                robot.flywheel.setPower(0);
                robot.flywheel2.setPower(0);
            }

            if(gamepad1.dpad_down) {
                flywheel1Adjust = flywheel1Adjust - 0.01;
                flywheel2Adjust = flywheel2Adjust + 0.01;
            } else if(gamepad1.dpad_up) {
                flywheel1Adjust = flywheel1Adjust + 0.01;
                flywheel2Adjust = flywheel2Adjust - 0.01;
            }

            if (flywheel1Adjust > 0.85) {
                flywheel1Adjust = 0.85;
                flywheel2Adjust = -0.85;
            } else if (flywheel1Adjust < 0) {
                flywheel1Adjust = 0;
                flywheel2Adjust = 0;
            }

//            wobble arm power with gamepad 2 right sticks (not ideal when driving because the
//            arm can be out of control when it comes to setting speeds w/ the gamepad)
//            if (gamepad2.right_stick_y != 0){
//                robot.wobbleArm.setPower(gamepad2.right_stick_y);
//            } else {
//                robot.wobbleArm.setPower(0);
//            }

//            if (gamepad2.right_stick_y != 0) {
//                robot.wobbleServo.setPosition(0);
//            } else {
//                robot.wobbleServo.setPosition(0.5);
//            }

            if(gamepad1.left_bumper) {
                robot.testServo.setPosition(robot.SERVO_GRAB);
            } else {
                robot.testServo.setPosition(robot.SERVO_RELEASE);
            }

//          commands to power the arm for the wobble goal manipulator
            if (gamepad2.dpad_up){
                robot.moveArm(1);
            } else if (gamepad2.dpad_down){
                robot.moveArm(-1);
            } else if (gamepad2.dpad_right) {
                robot.moveArm(0);
            }

            if (wobbleServoAdjust == 1) {
                robot.wobbleServo.setPosition(robot.SERVO_GRAB);
            } else {
                robot.wobbleServo.setPosition(robot.SERVO_RELEASE);
            }

            if (gamepad2.x) {
                wobbleServoAdjust = 1;
            } else if (gamepad2.y) {
                wobbleServoAdjust = 0;
            }

            telemetry.addData("wobbleServo pos: ", robot.wobbleServo.getPosition());
        }
    }
}