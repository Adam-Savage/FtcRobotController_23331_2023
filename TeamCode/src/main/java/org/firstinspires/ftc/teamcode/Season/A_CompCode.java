package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@Disabled
@TeleOp
public class A_CompCode extends LinearOpMode {

//---------------------------------------------------------------------------

    //Motor Speed
    public static final double ClimbSpeedUp = -0.6;
    public static final double ClimbSpeedDown = 1;

//---------------------------------------------------------------------------

    //Motor Set Points
    public static int LiftSetPtIntake = 0;
    public static int LiftSetPtLvl1 = 600;
    public static int LiftSetPtLvl2 = 1000;
    public static int LiftSetPtLvl3 = 1600;

    public static int ClimbSetPtUp = -2150;
//    public static int ClimbSetPtMid = -600;
    public static int ClimbSetPtDown = -10;

//---------------------------------------------------------------------------

    //Initialise Servo State
    public static boolean ClawOpen = false;
    public static boolean WristOut = false;

//---------------------------------------------------------------------------

    //Servo Set Points
    public static double WristSetPtIn = 0.34;
    public static double WristSetPtOut = 0.67;
    public static double WristSetPtScore = 0.48;

    public static double ClawSetPtClosed = 0.97;
    public static double ClawSetPtOpen = 0.88;
    public static double ClawSetPtSingleSmall = 1;
    public static double ClawSetPtSingleWide = 0.8;
    public static double ClawSetPtOpenSingleWide = 0.75;

    public static double DroneSetPtClosed = 1;
    public static double DroneSetPtOpen = 0;

    public static double HookSetPtClosed = 0.05;
    public static double HookSetPtOpen = 0.25;

//---------------------------------------------------------------------------

    //PIDF Variables
    private PIDController controller;

    public static double p = 0.004, i = 0.001, d = 0.0001;
    public static double f = 0.0001;

    public static int target = 0;

//---------------------------------------------------------------------------

    //Limit Switch Definition
    DigitalChannel limitSwitch;

//---------------------------------------------------------------------------

    //Auto Pick up sleeps
    public static int WristSleepDown = 400;
    public static int WristSleepUp = 0;
    public static int WristSleepDownSmall = 400;
    public static int WristSleepUpSmall = 200;
    public static int WristSleepBack = 100;

//---------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {

//---------------------------------------------------------------------------

        //Motor Declaration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Leftback");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Rightfront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Rightback");
        DcMotor Lift = hardwareMap.dcMotor.get("Lift");
        DcMotor Climb = hardwareMap.dcMotor.get("Climb");

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Driving Variables
        double LeftStickY;
        double LeftStickX;

//---------------------------------------------------------------------------

        //Encoder Mode
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Break
        Climb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

//---------------------------------------------------------------------------

        //Servo Declaration
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo Claw = hardwareMap.servo.get("Claw");
        Servo Drone = hardwareMap.servo.get("Drone");
        Servo Hook = hardwareMap.servo.get("Hook");

        //Initialise Servos
        Claw.setPosition(ClawSetPtClosed);
        Wrist.setPosition(WristSetPtIn);
        Drone.setPosition(DroneSetPtClosed);
        Hook.setPosition(HookSetPtClosed);

//---------------------------------------------------------------------------

        //Track Previous State of Buttons
        //Claw Control
        boolean previousRBumperButtonState = false;
        //Wrist Control
        boolean previousXButtonState = false;

//---------------------------------------------------------------------------

        //PIDF Setup
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Limit Switch Setup
        limitSwitch = hardwareMap.digitalChannel.get("LiftLimitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

//---------------------------------------------------------------------------

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

//---------------------------------------------------------------------------

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//---------------------------------------------------------------------------

            //Initialise Buttons
            //Claw Control
            boolean currentRBumperButtonState = gamepad1.right_bumper;
            //Wrist Control
            boolean currentXButtonState = gamepad1.x;

//---------------------------------------------------------------------------

            //Drive Control
            //Slow Driving
            if (gamepad1.left_bumper) {
                LeftStickY = -gamepad1.left_stick_y * 0.2;
                LeftStickX = gamepad1.left_stick_x * 0.2;
            } else {
                LeftStickY = -gamepad1.left_stick_y;
                LeftStickX = gamepad1.left_stick_x * 0.5;
            }

            //Mecanum Driving with Triggers
            if (gamepad1.left_trigger > 0.1) {
                //Strafe Left
                frontLeftMotor.setPower(-gamepad1.left_trigger);
                frontRightMotor.setPower(gamepad1.left_trigger);
                backLeftMotor.setPower(gamepad1.left_trigger);
                backRightMotor.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                //Strafe Right
                frontLeftMotor.setPower(gamepad1.right_trigger);
                frontRightMotor.setPower(-gamepad1.right_trigger);
                backLeftMotor.setPower(-gamepad1.right_trigger);
                backRightMotor.setPower(gamepad1.right_trigger);
            } else {
                //Normal POV Drive
                double drive = LeftStickY;
                double turn = LeftStickX;
                frontLeftMotor.setPower(Range.clip(drive + turn, -1.0, 1.0));
                backLeftMotor.setPower(Range.clip(drive + turn, -1.0, 1.0));
                frontRightMotor.setPower(Range.clip(drive - turn, -1.0, 1.0));
                backRightMotor.setPower(Range.clip(drive - turn, -1.0, 1.0));
            }

//---------------------------------------------------------------------------

            //Lift Control

            //Limit Switch Encoder Reset
//            if (limitSwitch.getState() == false) {
//                // Limit switch is pressed, reset the motor encoder
//                Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            //PIDF Loop
            controller.setPID(p, i, d);
            int LiftPos = Lift.getCurrentPosition();
            double pid = controller.calculate(LiftPos, target);
            double ff = target * f;

            double power = pid + ff;
            Lift.setPower(power);


            //A Button Pressed
            if (gamepad1.a) {
                Wrist.setPosition(WristSetPtScore);
                target = LiftSetPtLvl1;
                }

            //B Button Pressed
            else if (gamepad1.b) {
                Wrist.setPosition(WristSetPtScore);
                target = LiftSetPtLvl2;
                }

            //Y Button Pressed
            else if (gamepad1.y) {
                Wrist.setPosition(WristSetPtScore);
                target = LiftSetPtLvl3;
            }

//---------------------------------------------------------------------------

                //Climb Control
                //Dpad LEFT Pressed
            if (gamepad1.dpad_left) {
                //Open Servo
                Hook.setPosition(HookSetPtOpen);
                //Wait for servo to open
                sleep(500);
                //Set Target Position and Power
                Climb.setTargetPosition(ClimbSetPtUp);
                Climb.setPower(ClimbSpeedUp);
                //Set Run Mode
                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Wait for Target Position
                while (opModeIsActive() && Climb.isBusy()) {
                    telemetry.addLine("Climb Going Up");
                    telemetry.addData("Motor Position", Climb.getCurrentPosition());
                    telemetry.update();
                }

//                //Set Target Position and Power
//                Climb.setTargetPosition(ClimbSetPtDown);
//                Climb.setPower(ClimbSpeedDown);
//                //Set Run Mode
//                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //Wait for Target Position
//                while (opModeIsActive() && Climb.isBusy()) {
//                    telemetry.addLine("Climb Extending Hook");
//                    telemetry.addData("Climb Position", Climb.getCurrentPosition());
//                    telemetry.update();
//                }
//
//                //Set Target Position and Power
//                Climb.setTargetPosition(ClimbSetPtUp);
//                Climb.setPower(ClimbSpeedUp);
//                //Set Run Mode
//                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //Wait for Target Position
//                while (opModeIsActive() && Climb.isBusy()) {
//                    telemetry.addLine("Climb Going Up: Take 2");
//                    telemetry.addData("Motor Position", Climb.getCurrentPosition());
//                    telemetry.update();
//                }

                //Reset Power
                Climb.setPower(0);
                //Reset Run Mode
                Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            //Dpad Down Pressed
            else if (gamepad1.dpad_down) {
                //Set Target Position and Power
                Climb.setTargetPosition(ClimbSetPtDown);
                Climb.setPower(ClimbSpeedDown);
                //Set Run Mode
                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Wait for Target Position
                while (opModeIsActive() && Climb.isBusy()) {
                    telemetry.addLine("Climb Going Down");
                    telemetry.addData("Motor Position", Climb.getCurrentPosition());
                    telemetry.update();
                }
                //Reset Power
                Climb.setPower(0);
                //Reset Run Mode
                Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

//---------------------------------------------------------------------------

            //Wrist Toggle
            //Check if the button is currently pressed and was not pressed in the previous iteration
            if (currentXButtonState && !previousXButtonState) {

                //If Lift in scoring, let go and retract wrist
                if (Lift.getCurrentPosition() >= 100) {
                    //retract wrist & Close Claw
                    Claw.setPosition(ClawSetPtOpen);
                    sleep(WristSleepBack);
                    Wrist.setPosition(WristSetPtIn);
                    ClawOpen = true;
                    WristOut = false;
                    //Lift to position
                    target = LiftSetPtIntake;
                }

                //If Lift at 0, reach out and grab a pixel
                else if (Lift.getCurrentPosition() <100) {
                    Claw.setPosition(ClawSetPtOpen);
                    ClawOpen = true;
                    Wrist.setPosition(WristSetPtOut);
                    WristOut = true;
                    //Wrist Out
                    sleep(WristSleepDown);
                    Claw.setPosition(ClawSetPtClosed);
                    ClawOpen = false;
                    //pick up pixel
                    sleep(WristSleepUp);
                    //wait for claw to close
                    Wrist.setPosition(WristSetPtIn);
                    WristOut = false;
                    //Wrist In
                }
            }

//---------------------------------------------------------------------------

            //Manual Claw Toggle
            //Check if the button is currently pressed and was not pressed in the previous iteration
            if (currentRBumperButtonState && !previousRBumperButtonState) {
                if (ClawOpen) {
                    Claw.setPosition(ClawSetPtClosed);
                    ClawOpen = false;
                    //Claw Closed
                } else {
                    Claw.setPosition(ClawSetPtOpen);
                    ClawOpen = true;
                    //Claw Open
                }
            }

//---------------------------------------------------------------------------

            if (gamepad1.dpad_left) {


            }

//---------------------------------------------------------------------------

            //Auto Claw Toggle
            if (gamepad1.dpad_right && Lift.getCurrentPosition() < 100) {
                Claw.setPosition(ClawSetPtOpen);
                ClawOpen = true;
                Wrist.setPosition(WristSetPtOut);
                WristOut = true;
                //Wrist Out
                sleep(WristSleepDownSmall);
                Claw.setPosition(ClawSetPtSingleSmall);
                ClawOpen = false;
                //pick up pixel
                sleep(WristSleepUpSmall);
                //wait for claw to close
                Wrist.setPosition(WristSetPtIn);
                WristOut = false;
                //Wrist In
            }

            if (gamepad1.dpad_left) {
                Claw.setPosition(ClawSetPtOpenSingleWide);
                ClawOpen = true;
                Wrist.setPosition(WristSetPtOut);
                WristOut = true;
                //Wrist Out
                sleep(WristSleepDownSmall);
                Claw.setPosition(ClawSetPtSingleWide);
                ClawOpen = false;
                //pick up pixel
                sleep(WristSleepUpSmall);
                //wait for claw to close
                Wrist.setPosition(WristSetPtIn);
                WristOut = false;
                //Wrist In
            }

//---------------------------------------------------------------------------

            //Drone Control
            //If Drone is closed
            if (gamepad1.dpad_left && Drone.getPosition() < DroneSetPtOpen) {
                //Open Drone Servo
                Drone.setPosition(DroneSetPtOpen);
            }
            //If Drone is open
            else if (gamepad1.dpad_left && Drone.getPosition() > DroneSetPtClosed) {
                //Close Drone Servo
                Drone.setPosition(DroneSetPtClosed);
            }

//---------------------------------------------------------------------------

            //Update previous button states
            previousRBumperButtonState = currentRBumperButtonState;
            //Claw Button State
            previousXButtonState = currentXButtonState;
            //Wrist Button State

//---------------------------------------------------------------------------

            //Telemetry Update
            //Drive Information
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Strafe Left", gamepad1.left_trigger);
            telemetry.addData("Strafe Right", gamepad1.right_trigger);
            //Lift Information
            telemetry.addData("1.Lift Position", Lift.getCurrentPosition());
            telemetry.addData("2.Lift Target", target);
            telemetry.addData("Lift Limit Switch", limitSwitch.getState());
            //Climb Information
            telemetry.addData("5.Climb State", gamepad1.dpad_up ? "Up" : "Down");
            telemetry.addData("6.Climb Position", Climb.getCurrentPosition());
            //Claw Information
            telemetry.addData("3.Claw State", ClawOpen ? "Open" : "Closed");
            telemetry.addData("claw", Claw.getPosition());
            //Wrist Information
            telemetry.addData("4.Wrist State", WristOut ? "Out" : "In");
            telemetry.addData("wrist", Wrist.getPosition());
            //Update
            telemetry.update();


            }
        }
    }

