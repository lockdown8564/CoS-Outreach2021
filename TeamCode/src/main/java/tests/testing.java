package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.clip;


// before anything goes wrong lol (this class will prob be removed later -> just for testing out movements and stuff)

@TeleOp(name="robotTeleOp", group="Linear Opmode")
//@Disabled
public class testing extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightWheel");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftWheel");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightWheel");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
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



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontLeftWheel (%.2f)" + frontLeftPower);
            telemetry.addData("Motors", "frontRightWheel (%.2f)" + frontRightPower);
            telemetry.addData("Motors", "backLeftWheel (%.2f)" + backLeftPower);
            telemetry.addData("Motors", "backRightWheel (%.2f)"+ backRightPower);
            telemetry.update();
        }
    }
}
