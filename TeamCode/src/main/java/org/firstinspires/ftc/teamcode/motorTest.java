/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.clip;


@TeleOp(name="shooterMotorTest", group="Linear Opmode")
//@Disabled
public class motorTest extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;
    DcMotor testMotorDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        testMotorDrive = hardwareMap.get(DcMotor.class, "testMotor1");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightWheel");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftWheel");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightWheel");

        testMotorDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double testMotor1Power;
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double powerSide;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            frontLeftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
            frontRightPower   = Range.clip(drive + turn, -1.0, 1.0) ;
            backLeftPower = Range.clip(drive - turn, -1.0, 1.0) ;
            backRightPower = Range.clip(drive + turn, -1.0, 1.0) ;
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            // commands to move robot forward and backward


            powerSide = -gamepad1.left_stick_x;
            if (powerSide == -gamepad1.left_stick_x) {
                frontLeftDrive.setPower(-powerSide);
                frontRightDrive.setPower(powerSide);
                backLeftDrive.setPower(powerSide);
                backRightDrive.setPower(-powerSide);
            }
            // commands to move robot from side to side


            testMotor1Power  = -gamepad2.left_stick_y;
            testMotorDrive.setPower(testMotor1Power);
            // used to power motor for shooter



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "testMotor1 (%.2f)", testMotor1Power, " frontLeftPower (%.2f)", frontLeftPower, " frontRightPower (%.2f)", frontRightPower, " backLeftPower (%.2f)", backLeftPower, " backRightPower (%.2f)", backRightPower);
            telemetry.update();
        }
    }
}
