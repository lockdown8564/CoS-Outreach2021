package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "pid test run", group = "Auto")
public class pidController extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime pidRuntime = new ElapsedTime();

    double integral = 0;
    double repCount = 0;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(5, 5, 5);
    FtcDashboard dashboard;

    public static double TARGET_POS = 20;
    public static double SPEED = 0.1;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

//        if(opModeIsActive()){
//            pidDrive(0.3, TARGET_POS, TARGET_POS, TARGET_POS, TARGET_POS);
//        FlPidDrive(TARGET_POS);
//        FrPidDrive(TARGET_POS);
//        BlPidDrive(TARGET_POS);
//        BrPidDrive(TARGET_POS);
        PidDrive(0.2,5);
//        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pidRuntime.reset();
//        robot.stopAllMotorPower();
//        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        while(opModeIsActive()) {
//            telemetry.addData("reps: ", repCount);
//            telemetry.update();
//        }

//        FlPidDrive(0.4,  (int)(30*robot.getCountsPerInch()) + robot.frontLeftDrive.getCurrentPosition());
//        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
//        pidRuntime.reset();
//        robot.stopAllMotorPower();
//        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    // ignore
//    private void pidDrive(double speed, double FlInches, double FrInches, double BlInches, double BrInches){
//        int flTarget, frTarget, blTarget, brTarget;
//        if(opModeIsActive()){
////            double flPower =
//            flTarget = (int)(FlInches*robot.getCountsPerInch()) + robot.frontLeftDrive.getCurrentPosition();
//            frTarget = (int)(FrInches*robot.getCountsPerInch()) + robot.frontRightDrive.getCurrentPosition();
//            blTarget = (int)(BlInches*robot.getCountsPerInch()) + robot.backLeftDrive.getCurrentPosition();
//            brTarget = (int)(BrInches*robot.getCountsPerInch()) + robot.backRightDrive.getCurrentPosition();
//
//            // set target positions & power of each motor
//            robot.frontLeftDrive.setTargetPosition(flTarget);
//            robot.frontRightDrive.setTargetPosition(frTarget);
//            robot.backLeftDrive.setTargetPosition(blTarget);
//            robot.backRightDrive.setTargetPosition(brTarget);
//
//            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            FlPidDrive(flTarget);
//            FrPidDrive(frTarget);
//            BlPidDrive(blTarget);
////            BrPidDrive(brTarget);
//            pidRuntime.reset();
////          robot.verticalDriveSetTarget(LEFT_TARGET, RIGHT_TARGET);
//            robot.stopAllMotorPower();
//            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    public void FlPidDrive(/*double speed, */ double targetPos) {
//        double integral = 0;
//        ElapsedTime FlPidRuntime = new ElapsedTime();
//        while(opModeIsActive()) {
//            double error = robot.frontLeftDrive.getCurrentPosition();
//            double lastError = 0;
//            while (Math.abs(error) <= 9 && opModeIsActive()/* && repetitions < whatever number */) {
//                error = robot.frontLeftDrive.getCurrentPosition() - targetPos;
//                double changeInError = lastError - error;
//                integral += changeInError * FlPidRuntime.time();
//                double derivative = changeInError / FlPidRuntime.time();
//                double P = pidCoeffs.p * error;
//                double I = pidCoeffs.i * integral;
//                double D = pidCoeffs.d * derivative;
//                //maybe set target pos at top of this function
//                //maybe try troubleshooting by rearranging the setPower commands
//                robot.frontLeftDrive.setPower(/*(Math.abs(speed)) * */(P + I + D));
//                error = lastError;
//                repCount++;
//                FlPidRuntime.reset();
//            }
//            telemetry.addData("error: ", error);
//            telemetry.update();
//        }
//    }
//
//    public void FrPidDrive(/*double speed,*/ double targetPos) {
//        double integral = 0;
//        ElapsedTime FrPidRuntime = new ElapsedTime();
//        while(opModeIsActive()) {
//            double error = robot.frontRightDrive.getCurrentPosition();
//            double lastError = 0;
//            while (Math.abs(error) <= 9 && opModeIsActive()/* && repetitions < whatever number */) {
//                error = robot.frontRightDrive.getCurrentPosition() - targetPos;
//                double changeInError = lastError - error;
//                integral += changeInError * FrPidRuntime.time();
//                double derivative = changeInError / FrPidRuntime.time();
//                double P = pidCoeffs.p * error;
//                double I = pidCoeffs.i * integral;
//                double D = pidCoeffs.d * derivative;
//                robot.frontRightDrive.setPower(/*(Math.abs(speed)) * */(P + I + D));
//                error = lastError;
//                FrPidRuntime.reset();
//            }
//        }
//    }
//
//    public void BlPidDrive(/*double speed,*/ double targetPos) {
//        double integral = 0;
//        ElapsedTime BlPidRuntime = new ElapsedTime();
//        while(opModeIsActive()) {
//            double error = robot.backLeftDrive.getCurrentPosition();
//            double lastError = 0;
//            while (Math.abs(error) <= 9 && opModeIsActive()/* && repetitions < whatever number */) {
//                error = robot.backLeftDrive.getCurrentPosition() - targetPos;
//                double changeInError = lastError - error;
//                integral += changeInError * BlPidRuntime.time();
//                double derivative = changeInError / BlPidRuntime.time();
//                double P = pidCoeffs.p * error;
//                double I = pidCoeffs.i * integral;
//                double D = pidCoeffs.d * derivative;
//                robot.backLeftDrive.setPower(/*(Math.abs(speed)) * */(P + I + D));
//                error = lastError;
//                BlPidRuntime.reset();
//            }
//        }
//    }

    public void PidDrive(double speed, double distance) {
        repCount = 0;
        double targetPos = (int)(distance*robot.getCountsPerInch()) + (((robot.backRightDrive.getCurrentPosition()
                + robot.backLeftDrive.getCurrentPosition() + robot.frontRightDrive.getCurrentPosition()
                + robot.frontLeftDrive.getCurrentPosition()) / 4));
        if(opModeIsActive()) {
        double error = ((robot.backRightDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition()
            + robot.frontRightDrive.getCurrentPosition() + robot.frontLeftDrive.getCurrentPosition()) / 4);
        double lastError = 0;
            while (Math.abs(error) <= 9 && opModeIsActive() && repCount < targetPos) {
                error = robot.backRightDrive.getCurrentPosition() - targetPos;
                double changeInError = lastError - error;
                integral += changeInError * pidRuntime.time();
                double derivative = changeInError / pidRuntime.time();
                double P = pidCoeffs.p * error;
                double I = pidCoeffs.i * integral;
                double D = pidCoeffs.d * derivative;
                robot.setAllDrivePower((Math.abs(speed)) * (P + I + D));
                error = lastError;
                repCount++;
                pidRuntime.reset();
            }
            robot.stopAllMotorPower();
            error = ((robot.backRightDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition()
                    + robot.frontRightDrive.getCurrentPosition() + robot.frontLeftDrive.getCurrentPosition()) / 4);
            repCount = 0;
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}