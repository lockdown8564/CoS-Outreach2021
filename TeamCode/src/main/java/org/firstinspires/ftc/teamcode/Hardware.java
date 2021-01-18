package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
<<<<<<< HEAD
    private static Hardware robot = null;
    private static HardwareMap hwMap;

    public DcMotor frontLeftDrive=null;
    public DcMotor frontRightDrive=null;
    public DcMotor backLeftDrive=null;
    public DcMotor backRightDrive=null;
    public BNO055IMU imu=null;
    public Orientation angles=null;
    // public Servo testServo =null;

    public static Hardware getInstance(){
        if(robot == null){
            robot = new Hardware();
        }
        return robot;
    }
=======
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor intake;
    public Servo testServo;

    public BNO055IMU imu;
    public Orientation angles;
    public static final double SERVO_GRAB = 1;
    public static final double SERVO_RELEASE = 0.4;
>>>>>>> 330a7e602f6a23287b17da2dd0a9cc64f3a22514


    public ElapsedTime runtime = new ElapsedTime();

<<<<<<< HEAD

    private void init(HardwareMap hardwareMap){
        hwMap = hardwareMap;


        try{
            frontLeftDrive = hwMap.get(DcMotor.class, "frontLeftWheel");
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftDrive.setPower(0);
        }catch (Exception p_exception){
            frontLeftDrive = null;
        }


        try{
            frontRightDrive = hwMap.get(DcMotor.class, "frontRightWheel");
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setPower(0);
        }catch (Exception p_exception){
            frontRightDrive = null;
        }
        try{
            backLeftDrive = hwMap.get(DcMotor.class, "backLeftWheel");
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setPower(0);
        }catch (Exception p_exception){
            backLeftDrive = null;
        }

        try{
            backRightDrive =hwMap.get(DcMotor.class, "backRightWheel");
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setPower(0);
        }catch (Exception p_exception){
            backRightDrive = null;
        }
        /*
        try{
        testServo = hwMap.get(Servo.class, "tS");

        }catch (Exception p_exception){
            testServo = null;
        }
        */

=======
    public Hardware(){}
    public void initialize(HardwareMap hardwareMap){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightWheel");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftWheel");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightWheel");
        testServo = hardwareMap.get(Servo.class, "testServo");
        intake = hardwareMap.get(DcMotor.class, "intake");
>>>>>>> 330a7e602f6a23287b17da2dd0a9cc64f3a22514

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO55IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

<<<<<<< HEAD



=======
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
>>>>>>> 330a7e602f6a23287b17da2dd0a9cc64f3a22514


    }
}
