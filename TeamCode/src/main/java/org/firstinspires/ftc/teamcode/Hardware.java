package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    public Servo testServo;
    private double flPower, frPower, blPower, brPower = 0;


    private static final double TICKS_PER_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 0.66666667;
    private static final double WHEEL_DIAMETER_INCHES = 3.93700787402;
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double SERVO_GRAB = 1;
    public static final double SERVO_RELEASE = 0.4;

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
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRightDrive = hwMap.get(DcMotor.class, "frontRightWheel");
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backLeftDrive = hwMap.get(DcMotor.class, "backLeftWheel");
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive =hwMap.get(DcMotor.class, "backRightWheel");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake =hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testServo = hwMap.get(Servo.class, "testServo");

    }

    public enum DriveDirection{
        FORWARD,
        REVERSE
    }

    public void mecanumDrive(double vertical, double strafe, double horizontal, DriveDirection direction, double maxSpeed){
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

        flPower = vertical + strafe + horizontal;
        frPower = vertical - strafe - horizontal;
        blPower = vertical - strafe + horizontal;
        brPower = vertical + strafe - horizontal;
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

}
