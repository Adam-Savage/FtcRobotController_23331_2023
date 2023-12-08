package org.firstinspires.ftc.teamcode.Season.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Config
@TeleOp
public class A_TeleOp_Syd extends LinearOpMode {

//---------------------------------------------------------------------------

    //Motor Set Points
    public static int LiftSetPtIntake = -5;
    public static int LiftSetPtLvl1 = 400;
    public static int LiftSetPtLvl2 = 1000;
    public static int LiftSetPtLvl3 = 1600;
    public static int LiftSetPtLvl4 = 2200;

    public static int ClimbSetPtUp = -2150;
    public static int ClimbSetPtDown = 0;

//---------------------------------------------------------------------------

    //Servo Set Points
    public static double WristSetPtIn = 0.38;
    public static double WristSetPtOut = 0.64;
    public static double WristSetPtScore = 0.44;

    public static double ClawSetPtClosed = 0.88;
    public static double ClawSetPtOpen = 0.8;
    public static double ClawSetPtSingleSmall = 0.94;

    public static double DroneSetPtClosed = 0.5;
    public static double DroneSetPtOpen = 0.59;

    public static double HookSetPtClosed = 0.5;
    public static double HookSetPtOpen = 0.35;

//---------------------------------------------------------------------------

    //Lift PIDF Variables
    public PIDController Lift_controller;
    public static double Lift_p = 0.002, Lift_i = 0.00, Lift_d = 0.0001;
    public static double Lift_f = 0.000;
    public static int Lift_target = 0;

    //Climb PIDF Variables
    public PIDController Climb_controller;
    public static double Climb_p = 0.005, Climb_i = 0.02, Climb_d = 0;
    public static double Climb_f = 0;
    public static int Climb_target = 0;

//---------------------------------------------------------------------------

    //Limit Switch Definition
    DigitalChannel LiftLimitSwitch;
    DigitalChannel ClimbLimitSwitch;

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

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

        //IMU Declaration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

//---------------------------------------------------------------------------

        //Motor Declaration
        DcMotor Lift = hardwareMap.dcMotor.get("Lift");
        DcMotor Climb = hardwareMap.dcMotor.get("Climb");

        //Encoder Mode
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Break
        Climb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

//---------------------------------------------------------------------------

        //PIDF Setup
        Lift_controller = new PIDController(Lift_p, Lift_i, Lift_d);
        Climb_controller = new PIDController(Climb_p, Climb_i, Climb_d);

        //Limit Switch Setup
        LiftLimitSwitch = hardwareMap.digitalChannel.get("LiftLimitSwitch");
        LiftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        ClimbLimitSwitch = hardwareMap.digitalChannel.get("ClimbLimitSwitch");
        ClimbLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

//---------------------------------------------------------------------------

        //Initialise Servo State
        boolean ClawOpen = false;
        boolean WristOut = false;

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

        //Initialise PIDF Loops
        Climb_target = ClimbSetPtDown;
        Lift_target = LiftSetPtIntake;

//---------------------------------------------------------------------------

        //Track Previous State of Buttons

        //Wrist Control
        boolean previousXButtonState = false;
        boolean currentXButtonState;
        //2 Pixel Control
        double previousLTriggerState = 0;
        double currentLTriggerState;
        //1 Pixel Control
        double previousRTriggerState = 0;
        double currentRTriggerState;

//---------------------------------------------------------------------------

        //Send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

//---------------------------------------------------------------------------

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//---------------------------------------------------------------------------

            //Initialise Buttons

            //Wrist Control
            currentXButtonState = gamepad1.x;
            //2 Pixel Control
            currentLTriggerState = gamepad1.left_trigger;
            //1 Pixel Control
            currentRTriggerState = gamepad1.right_trigger;

//---------------------------------------------------------------------------

            //Drive Control

            //Slow Driving
            if (gamepad1.left_bumper) {
                LeftStickY = -gamepad1.left_stick_y * 0.2;
                LeftStickX = gamepad1.left_stick_x * 0.2;
                RX = gamepad1.right_stick_x * 0.3;
            }

            //Normal Driving
            else {
                LeftStickY = -gamepad1.left_stick_y;
                LeftStickX = gamepad1.left_stick_x;
                RX = gamepad1.right_stick_x * 0.8;
            }

//---------------------------------------------------------------------------

            //Field Centric Drive

            //Reset IMU when START Pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            //Get robot facing direction
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //Calculate rotation
            double rotX = LeftStickX * Math.cos(-botHeading) - LeftStickY * Math.sin(-botHeading);
            double rotY = LeftStickX * Math.sin(-botHeading) + LeftStickY * Math.cos(-botHeading);

            //Strafing Correction
            rotX = rotX * 1;

            //Maths for individual motor control
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(RX), 1);
            double frontLeftPower = (rotY + rotX + RX) / denominator;
            double backLeftPower = (rotY - rotX + RX) / denominator;
            double frontRightPower = (rotY - rotX - RX) / denominator;
            double backRightPower = (rotY + rotX - RX) / denominator;

            //Set Powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

//---------------------------------------------------------------------------

            //Lift Control

            //Limit Switch Encoder Reset
            if ((!LiftLimitSwitch.getState()) && (Lift.getCurrentPosition() != 0)) {
                // Limit switch is pressed, reset the motor encoder
                Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            //PIDF Loop
            Lift_controller.setPID(Lift_p, Lift_i, Lift_d);
            int LiftPos = Lift.getCurrentPosition();
            double Lift_pid = Lift_controller.calculate(LiftPos, Lift_target);
            double Lift_ff = Lift_target * Lift_f;

            double Lift_power = Lift_pid + Lift_ff;
            Lift.setPower(Lift_power);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //A Button Pressed
            if(gamepad1.a) {
                Wrist.setPosition(WristSetPtScore);
                Lift_target = LiftSetPtLvl1;
            }

            //B Button Pressed
            else if (gamepad1.b) {
                Wrist.setPosition(WristSetPtScore);
                Lift_target = LiftSetPtLvl2;
            }

            //Y Button Pressed
            else if (gamepad1.y) {
                Wrist.setPosition(WristSetPtScore);
                Lift_target = LiftSetPtLvl3;
            }

            //X Button Pressed
            else if (gamepad1.x) {
                Wrist.setPosition(WristSetPtScore);
                Lift_target = LiftSetPtLvl4;
            }

//---------------------------------------------------------------------------

            //Climb Control

            //Limit Switch Encoder Reset
            if ((!ClimbLimitSwitch.getState()) && (Climb.getCurrentPosition() != 0)) {
                // Limit switch is pressed, reset the motor encoder
                Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            //PIDF Loop
            Climb_controller.setPID(Climb_p, Climb_i, Climb_d);
            int ClimbPos = Climb.getCurrentPosition();
            double Climb_pid = Climb_controller.calculate(ClimbPos, Climb_target);
            double Climb_ff = Climb_target * Climb_f;

            double Climb_power = Climb_pid + Climb_ff;
            Climb.setPower(Climb_power);


            //Climb extend
            if (gamepad1.dpad_left && gamepad1.right_bumper) {
                //Open Servo
                Hook.setPosition(HookSetPtOpen);
                //Wait for servo to open
                sleep(500);
                //Set Target
                Climb_target = ClimbSetPtUp;
            }

            //Climb Retract
            else if (gamepad1.dpad_down && gamepad1.right_bumper) {
                //Retract climb and servo
                Climb_target = ClimbSetPtDown;
                Hook.setPosition(HookSetPtClosed);
                //Reset Drone Servo
                Drone.setPosition(DroneSetPtClosed);
            }
            Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//---------------------------------------------------------------------------

            //Drone Deployment

            //Deploy climb
            if (gamepad1.dpad_right && Climb.getCurrentPosition() > -100 &&
                    gamepad1.right_bumper) {
                //Send climb up with hook still retracted
                Climb_target = ClimbSetPtUp;
                sleep(500);
            }

            //Shoot Drone and retract
            else if (gamepad1.dpad_right && Climb.getCurrentPosition() < -100 &&
                    gamepad1.right_bumper) {
                //Shoot drone
                Drone.setPosition(DroneSetPtOpen);
            }

//---------------------------------------------------------------------------

            //2 Pixel Toggle

            //If Lift in scoring, let go and retract wrist
            if (Lift.getCurrentPosition() >= 100 && gamepad1.right_trigger > 0.1) {
                //retract wrist & Close Claw
                Claw.setPosition(ClawSetPtOpen);
                sleep(WristSleepBack);
                Wrist.setPosition(WristSetPtIn);
                ClawOpen = true;
                WristOut = false;
                //Lift to position
                Lift_target = LiftSetPtIntake;
            }

            //If lift in intake, start intake sequence
            else if (Lift.getCurrentPosition() < 100) {
                //Check if button has been pressed
                if (currentRTriggerState > 0 && previousRTriggerState <= 0) {
                    if (!WristOut) {
                        //Reach out and grab pixel
                        //Claw open
                        Claw.setPosition(ClawSetPtOpen);
                        ClawOpen = true;
                        //Wrist out
                        Wrist.setPosition(WristSetPtOut);
                        WristOut = true;
                        //Wait
                        sleep(WristSleepDown);
                        //Grab Pixel
                        Claw.setPosition(ClawSetPtClosed);
                        ClawOpen = false;
                        //Wait
                        sleep(WristSleepUp);
                    }
                }

                else if (currentRTriggerState <= 0 && previousRTriggerState > 0) {
                    //Pull pixel into robot
                    //Wrist in
                    Wrist.setPosition(WristSetPtIn);
                    WristOut = false;
                }
            }

//---------------------------------------------------------------------------

            //1 Pixel Toggle

            //If Lift in scoring, let go and retract wrist
            if (Lift.getCurrentPosition() >= 100 && gamepad1.left_trigger > 0.1) {
                //retract wrist & Close Claw
                Claw.setPosition(ClawSetPtOpen);
                sleep(WristSleepBack);
                Wrist.setPosition(WristSetPtIn);
                ClawOpen = true;
                WristOut = false;
                //Lift to position
                Lift_target = LiftSetPtIntake;
            }

            //If Lift in intake, enable intake sequence
            else if (Lift.getCurrentPosition() < 100) {
                //Check if the button has been pressed
                if (currentLTriggerState > 0 && previousLTriggerState <= 0) {
                    //Stop double deployment
                    if (!WristOut) {
                        //Reach out and grab pixel
                        //Claw open
                        Claw.setPosition(ClawSetPtOpen);
                        ClawOpen = true;
                        //Wrist out
                        Wrist.setPosition(WristSetPtOut);
                        WristOut = true;
                        //Wait
                        sleep(WristSleepDownSmall);
                        //Grab Pixel
                        Claw.setPosition(ClawSetPtSingleSmall);
                        ClawOpen = false;
                        //Wait
                        sleep(WristSleepUpSmall);
                    }
                }

                //Check if the button has been released
                else if (currentLTriggerState <= 0 && previousLTriggerState > 0) {
                    //Pull pixel into robot
                    //Wrist in
                    Wrist.setPosition(WristSetPtIn);
                    WristOut = false;
                }
            }

//---------------------------------------------------------------------------

//            //Manual Claw Toggle
//
//            //Check if the button is currently pressed and was not pressed in the previous iteration
//            if (currentXButtonState && !previousXButtonState) {
//                if (ClawOpen) {
//                    Claw.setPosition(ClawSetPtClosed);
//                    ClawOpen = false;
//                    //Claw Closed
//                } else {
//                    Claw.setPosition(ClawSetPtOpen);
//                    ClawOpen = true;
//                    //Claw Open
//                }
//            }

//---------------------------------------------------------------------------

            //Update previous button states

            //Wrist Button State
            previousXButtonState = currentXButtonState;
            //2 Pixel Control
            previousLTriggerState = currentLTriggerState;
            //1 Pixel Control
            previousRTriggerState = currentRTriggerState;

//---------------------------------------------------------------------------

            //Telemetry Update
            //Drive Information
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            //Lift Information
            telemetry.addData("1.Lift Position", Lift.getCurrentPosition());
            telemetry.addData("2.Lift Target", Lift_target);
            telemetry.addData("Lift Limit Switch", LiftLimitSwitch.getState());
            //Climb Information
            telemetry.addData("3.Climb Position", Climb.getCurrentPosition());
            telemetry.addData("4.Climb Target", Climb_target);
            telemetry.addData("5.Climb Limit Switch", ClimbLimitSwitch.getState());
            //Claw Information
            telemetry.addData("6.Claw State", ClawOpen ? "Open" : "Closed");
            telemetry.addData("claw", Claw.getPosition());
            //Wrist Information
            telemetry.addData("7.Wrist State", WristOut ? "Out" : "In");
            telemetry.addData("wrist", Wrist.getPosition());
            //Update
            telemetry.update();
        }
    }
}

