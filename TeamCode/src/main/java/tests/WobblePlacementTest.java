package tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="WobblePlacementTest", group="Linear Opmode")
@Disabled
public class WobblePlacementTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // NEEDS TO FIX VARIABLE IMPORTS AND HARDWARE MAP. ALSO NEEDS TO MAKE ALLIGN METHOD THAT ALLIGNS THE ROBOT IN THE RIGHT BOX BEFORE GOING FOWARD. ALSO HOW TO MOVE FOWARD.
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            //takes position from opencv and matches situation
            if (position == ONE) {
                toB();
            } else if (position == FOUR) {
                toC();
            } else {
                toA();
            }
            //functions to get the robot to the spot
            public static void toA() {
                //align code here
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
                sleep();
                //place wobble code here
            }
            public static void toB() {
                //align
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
                sleep();
                //place wobble code here


            }
            public static void toC() {
                //align
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
                sleep();
                //place wobble code here

            }
        }
    }
}
