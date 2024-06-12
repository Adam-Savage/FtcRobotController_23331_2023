package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp
public class A_TeleOp_APOC extends OpMode {

//---------------------------------------------------------------------------

    //Motor Set Points

    //Servo Set Points
    double leftArmStowed = 0.1;
    double leftArmPickUp = 0;
    double leftArmScore = 0.6;
    double rightArmStowed = 1 - leftArmStowed;
    double rightArmPickUp = 1 - leftArmPickUp;
    double rightArmScore = 1 - leftArmScore;

    int wristStowed = 0;
    int wristScore = 0;

    int spinClawStowed = 0;
    int spinClawLeftScore = 0;
    int spinClawRightScore = 0;

    int clawOpen = 0;
    int clawClosed = 0;
    int clawClosed1Pixel = 0;

    double intakeStowed = 0;
    double intakePickUp = 0.4;
    int intakePickUpStack = 0;



    //Autonomous Function Sleeps

//---------------------------------------------------------------------------

    //Elevator PIDF Variables
//    public PIDFController elevatorController;
//    public static double eP = 0, eI = 0, eD = 0;
//    public static double eF = 0;
//    public static int elevatorTarget = 0;

    //Climb PIDF Variables
//    public PIDFController climbController;
//    public static double cP = 0, cI = 0, cD = 0;
//    public static double cF = 0;
//    public static int climbTarget = 0;

    //Wrist PIDF Variables
//    public PIDFController wristController;
//    public static double wP = 0, wI = 0, wD = 0;
//    public static double wF = 0;
//    public static int wristTarget = 0;

//---------------------------------------------------------------------------

    //Limit Switch Definition
//    DigitalChannel elevatorLimit;
//    DigitalChannel innerPixel;
//    DigitalChannel outerPixel;
//    DigitalChannel climbHookLeft;
//    DigitalChannel climbHookRight;

//---------------------------------------------------------------------------

    //Trigger state initialization
//    public double previousRTriggerState = 0;
//    public double previousLTriggerState = 0;

    //Lift Reset timer initialization
    public long lastResetTime = 0;

//---------------------------------------------------------------------------

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    public DcMotor elevator;
    public DcMotor elevatorBack;
//    public DcMotor leftClimb;
//    public DcMotor rightClimb;
    public DcMotor intake;


    public Servo leftArm;
    public Servo rightArm;
//    public Servo wrist;
//    public Servo spinClaw;
//    public Servo openClaw;
    public Servo extendIntake;
//    public Servo deployDrone;
//    public Servo launchDrone;
//    public Servo leftClimbQuick;
//    public Servo rightClimbQuick;


//---------------------------------------------------------------------------

    @Override
    public void init() {
        //Code to run ONCE when the driver hits INIT

//---------------------------------------------------------------------------

        //Motor Declaration
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //Motor Reverse
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //IMU Declaration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

//---------------------------------------------------------------------------

        //Motor Declaration
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevatorBack = hardwareMap.get(DcMotor.class, "elevatorBack");
//        leftClimb = hardwareMap.get(DcMotor.class, "leftClimb");
//        rightClimb = hardwareMap.get(DcMotor.class, "rightClimb");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Encoder Mode
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Break
        elevator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        elevatorBack.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        leftClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        rightClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //Reverse
        elevatorBack.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //PIDF Setup
//        elevatorController = new PIDFController(eP, eI, eD, eF);
//        climbController = new PIDFController(cP, cI, cD, cF);
//        wristController = new PIDFController(wP, wI, wD, wF);

        //Servo Setup
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        spinClaw = hardwareMap.get(Servo.class, "spinClaw");
//        openClaw = hardwareMap.get(Servo.class, "openClaw");
        extendIntake = hardwareMap.get(Servo.class, "extendIntake");
//        deployDrone = hardwareMap.get(Servo.class, "deployDrone");
//        launchDrone = hardwareMap.get(Servo.class, "launchDrone");
//        leftClimbQuick = hardwareMap.get(Servo.class, "leftClimbQuick");
//        rightClimbQuick = hardwareMap.get(Servo.class, "rightClimbQuick");

        //Limit Switch Setup
//        elevatorLimit = hardwareMap.get(DigitalChannel.class, "elevatorLimit");
//        innerPixel = hardwareMap.get(DigitalChannel.class, "innerPixel");
//        outerPixel = hardwareMap.get(DigitalChannel.class, "outerPixel");
//        ClimbHookLeft = hardwareMap.get(DigitalChannel.class, "ClimbHookLeft");
//        ClimbHookRight = hardwareMap.get(DigitalChannel.class, "ClimbHookRight");

//---------------------------------------------------------------------------

        //Initialise Servos
        leftArm.setPosition(leftArmStowed);
        rightArm.setPosition(rightArmStowed);
//        wrist.setPosition(wristStowed);
//        spinClaw.setPosition(spinClawStowed);
//        openClaw.setPosition(clawOpen);
        extendIntake.setPosition(intakeStowed);
//        deployDrone.setPosition(0);
//        launchDrone.setPosition(0);
//        leftClimbQuick.setPosition(0);
//        rightClimbQuick.setPosition(0);

        //Initialise PIDF Loops
//        elevatorTarget = 0;
//        climbTarget = 0;
//        wristTarget = 0;

//---------------------------------------------------------------------------

        //Send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void init_loop() {
        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        //Robot state variables
        boolean Intaking = false;
        boolean Scoring = false;
        boolean Climbing = false;

        //Trigger tracking
        double currentLTriggerState = gamepad1.left_trigger;
        double currentRTriggerState = gamepad1.right_trigger;
        double current2ndLTriggerState = gamepad2.left_trigger;
        double current2ndRTriggerState = gamepad2.right_trigger;

//---------------------------------------------------------------------------
        //Variables

        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

//---------------------------------------------------------------------------

        //Drive Control

        //Slow Driving
        if (Intaking || Scoring || Climbing) {
            LeftStickY = -gamepad1.left_stick_y * 0.25;
            LeftStickX = gamepad1.left_stick_x * 0.25;
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

        IMU imu = hardwareMap.get(IMU.class, "imu");

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
        rotX = rotX * 1.2;

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

        //Elevator Control
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(-1 * gamepad2.left_stick_y);
        elevatorBack.setPower(-1 * gamepad2.left_stick_y);

        //MAX point for elevator = 3300 ticks

//        if (gamepad1.a) {
//            elevator.setTargetPosition(0);
//            elevatorBack.setTargetPosition(0);
//
//            elevator.setPower(0.2);
//            elevatorBack.setPower(0.2);
//
//            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            elevatorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        if (gamepad1.b) {
//            elevator.setTargetPosition(100);
//            elevatorBack.setTargetPosition(100);
//
//            elevator.setPower(0.2);
//            elevatorBack.setPower(0.2);
//
//            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            elevatorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        if (gamepad1.y) {
//            elevator.setTargetPosition(200);
//            elevatorBack.setTargetPosition(200);
//
//            elevator.setPower(0.2);
//            elevatorBack.setPower(0.2);
//
//            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            elevatorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        if (gamepad1.x) {
//            elevator.setTargetPosition(300);
//            elevatorBack.setTargetPosition(300);
//
//            elevator.setPower(0.2);
//            elevatorBack.setPower(0.2);
//
//            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            elevatorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }


//---------------------------------------------------------------------------

        //Intake Control
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.right_bumper) {
            Intaking = true;
            extendIntake.setPosition(intakePickUp);
            intake.setPower(1);
        }

        else {
            Intaking = false;
            extendIntake.setPosition(intakeStowed);
            intake.setPower(0.3);
        }
//---------------------------------------------------------------------------

        //Handoff

        //Close Claw
        if (gamepad2.left_bumper) {
//            openClaw.setPosition(clawClosed);
        }

        //Open Claw
        if (gamepad2.right_bumper) {
//            openClaw.setPosition(clawOpen);
        }

        //Pickup Arm
        if (gamepad2.a) {
            leftArm.setPosition(leftArmPickUp);
            rightArm.setPosition(rightArmPickUp);
        }

        //Score Arm
        if (gamepad2.b) {
            leftArm.setPosition(leftArmScore);
            rightArm.setPosition(rightArmScore);
        }

        //Stow Arm
        if (gamepad2.x) {
            leftArm.setPosition(leftArmStowed);
            rightArm.setPosition(rightArmStowed);
        }

//---------------------------------------------------------------------------

        //Update Variables

        //Update previous button states
//        previousLTriggerState = currentLTriggerState;
//        previousRTriggerState = currentRTriggerState;

//---------------------------------------------------------------------------

        //Telemetry Update
        //Drive Information
        telemetry.addData("Left Stick X:", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y:", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X:", gamepad1.right_stick_x);
        telemetry.addData("Elevator:", elevator.getCurrentPosition());
        telemetry.addData("Elevator Target:", elevator.getTargetPosition());
        telemetry.addData("ElevatorBack:", elevatorBack.getCurrentPosition());
        telemetry.addData("Elevator Back Target:", elevatorBack.getTargetPosition());
        //Update
        telemetry.update();

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

    @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP

//---------------------------------------------------------------------------

        //code here

//---------------------------------------------------------------------------

    }

//---------------------------------------------------------------------------

//Functions Go Here

    private void sleep (int t) {
        try {
            Thread.sleep(t);
        } catch(InterruptedException e){
            throw new RuntimeException(e);
        }
    }

//---------------------------------------------------------------------------

}