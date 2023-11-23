package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@Autonomous
public class Red_Wing extends LinearOpMode {

//---------------------------------------------------------------------------

    //Declare Drive Variables
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private ElapsedTime runtime = new ElapsedTime();

    //Calculate the COUNTS_PER_INCH for your specific drive train.
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

//---------------------------------------------------------------------------

    //Initialise Servo State
    public static boolean ClawOpen = false;
    public static boolean WristOut = false;

//---------------------------------------------------------------------------

    //Servo Set Points
    public static double WristSetPtIn = 0.34;
    public static double WristSetPtOut = 0.66;
    public static double WristSetPtScore = 0.48;

    public static double ClawSetPtClosed = 0.97;
    public static double ClawSetPtOpen = 0.88;

//---------------------------------------------------------------------------

    //Lift Things
    public static int LiftSetPtIntake = 0;
    public static int LiftSetPtLvl1 = 600;
    public static int LiftSetPtLvl2 = 1000;
    public static int LiftSetPtLvl3 = 1600;

    private DcMotor Lift = null;

//---------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//---------------------------------------------------------------------------

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");

        //Initialise Servos
        Claw.setPosition(ClawSetPtClosed);
        Wrist.setPosition(WristSetPtIn);

//---------------------------------------------------------------------------

        //Initialize Drive Variables
        frontLeft = hardwareMap.get(DcMotor.class, "Leftfront");
        backLeft = hardwareMap.get(DcMotor.class, "Leftback");
        frontRight = hardwareMap.get(DcMotor.class, "Rightfront");
        backRight = hardwareMap.get(DcMotor.class, "Rightback");

        //Motor Reverse
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Encoder Reset
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Motor Run Mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Lift Things
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//---------------------------------------------------------------------------

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();

//---------------------------------------------------------------------------

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//---------------------------------------------------------------------------

        //Red Wing Side
        encoderDrive(DRIVE_SPEED, 15, 15, 5.0);
        encoderDrive(DRIVE_SPEED, -6, -6, 5.0);

//---------------------------------------------------------------------------

            telemetry.addData("Path", "Complete");
            telemetry.addData("Lift Position", Lift.getCurrentPosition());
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }

//---------------------------------------------------------------------------

        public void encoderDrive ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                backLeft.setTargetPosition(newLeftTarget);
                frontLeft.setTargetPosition(newLeftTarget);
                backRight.setTargetPosition(newRightTarget);
                frontRight.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                backLeft.setPower(Math.abs(speed));
                frontLeft.setPower(Math.abs(speed));
                backRight.setPower(Math.abs(speed));
                frontRight.setPower(Math.abs(speed));

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (backLeft.isBusy() && backRight.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Currently at", " at %7d :%7d",
                            backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                // Turn off RUN_TO_POSITION
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // optional pause after each move.
                sleep(250);
            }
        }

        public void liftMove ( double speed, double height){

            if (opModeIsActive()) {
                Lift.setTargetPosition((int) height);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(speed);
                while (opModeIsActive() && Lift.isBusy()) {
                    telemetry.addData("Running to", height);
                    telemetry.addData("Currently at", Lift.getCurrentPosition());
                    telemetry.update();
                }
                Lift.setPower(0);
                Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
