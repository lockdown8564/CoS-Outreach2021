package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


import java.util.Arrays;

public class Hardware {
// note: don't change hardware class for now, all the motor hardware code is good so far

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor intake;
    public BNO055IMU imu;
    public Orientation angles;
    public DcMotor wobbleArm;
    public Servo wobbleServo;
    public DcMotor flywheel;
    public DcMotor flywheel2;
    public Servo shooterServo;
    int newWobbleArmTargetPos;
    public Servo testServo; // to test that one avox servo
//    public final static double STARTING_WOBBLE_SERVO_POS = 0.0;
//    public final static double WOBBLE_SERVO_MIN_RANGE = 0.5;
//    public final static double WOBBLE_SERVO_MAX_RANGE = 1.0;

    private double flPower, frPower, blPower, brPower = 0;
    private double previousAngle = 0;
    static final double P_TURN_COEFF = 0.1;
    final double P_DRIVE_COEFF = 0.1;
    public double wobbleArmMotor90 = 2912/4;
    public double wobbleArmMotorPower = 0.3;
    public double wobbleArmMotor45 = 2912/7;


    private static final double TICKS_PER_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 0.66666667;
    private static final double WHEEL_DIAMETER_INCHES = 3.93700787402;
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double SERVO_GRAB = 0;
    public static final double SERVO_RELEASE = 0.3;

    HardwareMap hwMap;

    public Hardware (){}

    public void init(HardwareMap hwMap){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftDrive = hwMap.get(DcMotor.class, "frontLeftWheel");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRightDrive = hwMap.get(DcMotor.class, "frontRightWheel");
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive = hwMap.get(DcMotor.class, "backLeftWheel");
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive = hwMap.get(DcMotor.class, "backRightWheel");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm = hwMap.get(DcMotor.class, "wobbleArm");
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel = hwMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel2 = hwMap.get(DcMotor.class, "flywheel2");
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        shooterServo = hwMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0);
        testServo = hwMap.get(Servo.class, "testServo");

    }

    public enum DriveDirection{
        FORWARD,
        REVERSE
    }

    private double intZ() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return (previousAngle - angles.firstAngle);
    }

    public void mecanumDrive(double vertical, double strafe, double turn, DriveDirection direction, double maxSpeed){
        double num = 1;
        switch(direction){
            case FORWARD:{
                break;
            }
            case REVERSE:{
                num = -1;
                break;
            }
        }

        flPower = vertical + strafe + turn;
        frPower = vertical - strafe - turn;
        blPower = vertical - strafe + turn;
        brPower = vertical + strafe - turn;
        speed(flPower * num, frPower * num, blPower * num,
                brPower * num, maxSpeed);

    }

    public void driveSetPower(double flPower, double frPower, double blPower, double brPower){
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    private void speed(double frontLeftPower, double frontRightPower, double backLeftPower,
                       double backRightPower, double maxVal){
        double[] motorPowers = {1.0, frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        Arrays.sort(motorPowers);
        double greatest = motorPowers[4];

        driveSetPower((frontLeftPower/greatest) * maxVal, (frontRightPower/greatest) * maxVal,
                (backLeftPower/greatest) * maxVal, (backRightPower/greatest) * maxVal);
    }

    double getCountsPerInch() {
        return COUNTS_PER_INCH;
    }

    public void moveArm(int direction) {
        if(direction == 1) {
            newWobbleArmTargetPos = /*wobbleArm.getCurrentPosition() +*/ -200;
            wobbleArm.setTargetPosition(newWobbleArmTargetPos);
            wobbleArm.setPower(wobbleArmMotorPower);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction == -1) {
            newWobbleArmTargetPos = /*wobbleArm.getCurrentPosition() - */ -1350;
            wobbleArm.setTargetPosition(newWobbleArmTargetPos);
            wobbleArm.setPower(wobbleArmMotorPower);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            newWobbleArmTargetPos = /*wobbleArm.getCurrentPosition() -*/ -650;
            wobbleArm.setTargetPosition(newWobbleArmTargetPos);
            wobbleArm.setPower(wobbleArmMotorPower);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    public void stopAllMotorPower() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(0);
        flywheel.setPower(0);
        flywheel2.setPower(0);
    }

    boolean driveIsBusy() {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() &&
                backRightDrive.isBusy();
    }

    void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    void verticalDriveSetTarget(int leftTarget, int rightTarget) {
        frontLeftDrive.setTargetPosition(leftTarget);
        frontRightDrive.setTargetPosition(rightTarget);
        backLeftDrive.setTargetPosition(leftTarget);
        backRightDrive.setTargetPosition(rightTarget);
    }

    void strafeDriveSetTarget(int flTarget, int frTarget, int blTarget, int brTarget) {
        frontLeftDrive.setTargetPosition(flTarget);
        frontRightDrive.setTargetPosition(frTarget);
        backLeftDrive.setTargetPosition(blTarget);
        backRightDrive.setTargetPosition(brTarget);
    }

    void sideDriveSetTarget(int direction, int leftTarget, int rightTarget) {
        if (direction == 1) {
            frontLeftDrive.setTargetPosition(leftTarget);
            frontRightDrive.setTargetPosition(-rightTarget);
            backLeftDrive.setTargetPosition(-leftTarget);
            backRightDrive.setTargetPosition(rightTarget);
        } else if (direction == -1) {
            frontLeftDrive.setTargetPosition(-leftTarget);
            frontRightDrive.setTargetPosition(rightTarget);
            backLeftDrive.setTargetPosition(leftTarget);
            backRightDrive.setTargetPosition(-rightTarget);
        }
    }

    void setAllDrivePower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    void setPidDrivePower(double flPower, double frPower, double blPower, double brPower){
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    public double getErrorForPID(double targetAngle) {
        double robotError;
        robotError = targetAngle - intZ();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError +=360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



}