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
    int elevatorStowed = -500;
    int elevatorLvl1 = -1000;
    int elevatorLvl2 = -1500;
    int elevatorLvl3 = -2000;
    //Max 3300

    int climbDeploy = -2600;
    int climbQuickUp = -2000;
    int climbRetract = 0;
    //Max -2600

    //Motor Powers
    double intakeIntaking = 1;
    double intakeHold = 0.3;

    //Servo Set Points
    double leftArmStowed = 0.6; //0.8
    double leftArmPickUp = 0.9;
    double leftArmScore = 0.2;
    double rightArmStowed = 1 - leftArmStowed;
    double rightArmPickUp = 1 - leftArmPickUp;
    double rightArmScore = 1 - leftArmScore;

    double wristStowed = 0;
    double wristScore = 0;
    double wristScoreLow = 0;

    double spinClawStowed = 0.6;
    double spinClawLeft = 1;
    double spinClawLeft45 = 0.77;
    double spinClawRight = 0;
    double spinClawRight45 = 0.4;

    double clawOpen = 0.7;
    double clawClosed = 0.85;
    double clawClosed1Pixel = 0.9;

    double intakeStowed = 0;
    double intakePickUp = 0.4;
    double intakePickUpStack = 0;

    //Autonomous Function Sleeps

    int clawGrabBeforeIntakeUp = 0;

//---------------------------------------------------------------------------

    //Elevator PIDF Variables
    public PIDFController elevatorController;
    public static double eP = 0.002, eI = 0.00, eD = 0.0001;
    public static double eF = 0;
    public static int elevatorTarget;

    //Climb PIDF Variables
    public PIDFController leftClimbController;
    public PIDFController rightClimbController;
    public static double cP = 0.018, cI = 0.1, cD = 0.00001;
    public static double cF = 0;
    public static int climbTarget;


    //Wrist PIDF Variables
//    public PIDFController leftWristController;
//    public PIDFController rightWristController;
//    public static double wP = 0, wI = 0, wD = 0;
//    public static double wF = 0;
//    public static int wristTarget;

//---------------------------------------------------------------------------

    //Trigger state initialization
    public double previousLTriggerState = 0;
    public double previousRTriggerState = 0;
    public double previous2ndLTriggerState = 0;
    public double previous2ndRTriggerState = 0;

    //Lift Reset timer initialization
    public long lastResetTime = 0;

//---------------------------------------------------------------------------

    //Motor Definition
    //Control Hub
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //Expansion Hub
    public DcMotor leftClimb;
    public DcMotor rightClimb;
    public DcMotor elevator;
    public DcMotor intake;

    //Servo Definition
    //Control Hub
    public Servo extendIntake;
    public Servo leftArm;
    public Servo rightArm;
    public Servo wrist;
    public Servo spinClaw;
    public Servo claw;

    //Expansion Hub
    public Servo leftClimbQuick;
    public Servo rightClimbQuick;
    public Servo deployDrone;
    public Servo launchDrone;

    //Limit Switch Definition
    //Control Hub
    DigitalChannel hookLeft;
    DigitalChannel innerPixel;
    DigitalChannel outerPixel;

    //Expansion Hub
    DigitalChannel hookRight;
    DigitalChannel elevatorLimit;

//---------------------------------------------------------------------------

    @Override
    public void init() {
        //Code to run ONCE when the driver hits INIT

//---------------------------------------------------------------------------

        //Motor Declaration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Motor Reverse
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //IMU Declaration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

//---------------------------------------------------------------------------

        //Motor Declaration
        leftClimb = hardwareMap.get(DcMotor.class, "leftClimb");
        rightClimb = hardwareMap.get(DcMotor.class, "rightClimb");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Encoder Mode
        leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Enable Break
        leftClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightClimb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //Reverse
        leftClimb.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

//---------------------------------------------------------------------------

        //PIDF Setup
        elevatorController = new PIDFController(eP, eI, eD, eF);
        leftClimbController = new PIDFController(cP, cI, cD, cF);
        rightClimbController = new PIDFController(cP, cI, cD, cF);
//        wristController = new PIDFController(wP, wI, wD, wF);

        //Servo Setup
        extendIntake = hardwareMap.get(Servo.class, "extendIntake");
        leftClimbQuick = hardwareMap.get(Servo.class, "leftClimbQuick");
        deployDrone = hardwareMap.get(Servo.class, "deployDrone");
        launchDrone = hardwareMap.get(Servo.class, "launchDrone");

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        wrist = hardwareMap.get(Servo.class, "rightClimbQuick");
        spinClaw = hardwareMap.get(Servo.class, "spinClaw");
        claw = hardwareMap.get(Servo.class, "claw");
        rightClimbQuick = hardwareMap.get(Servo.class, "rightClimbQuick");

        //Limit Switch Setup
        hookLeft = hardwareMap.get(DigitalChannel.class, "hookLeft");
        innerPixel = hardwareMap.get(DigitalChannel.class, "innerPixel");
        outerPixel = hardwareMap.get(DigitalChannel.class, "outerPixel");

        hookRight = hardwareMap.get(DigitalChannel.class, "hookRight");
        elevatorLimit = hardwareMap.get(DigitalChannel.class, "elevatorLimit");

//---------------------------------------------------------------------------

        //Initialise Servos
        extendIntake.setPosition(intakeStowed);
        leftArm.setPosition(leftArmStowed);
        rightArm.setPosition(rightArmStowed);
        wrist.setPosition(wristStowed);
        spinClaw.setPosition(spinClawStowed);
        claw.setPosition(clawOpen);

        leftClimbQuick.setPosition(0);
        rightClimbQuick.setPosition(0);
        deployDrone.setPosition(0);
        launchDrone.setPosition(0);

        //Initialise PIDF Loops
        elevatorTarget = 0;
        climbTarget = climbRetract;
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

//        spinClaw.setPosition(spinClawStowed);
//        wrist.setPosition(wristStowed);
//        leftArm.setPosition(leftArmStowed);
//        rightArm.setPosition(rightArmStowed);

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
        //PIDF Loops
        //Elevator
        elevatorController.setPIDF(eP, eI, eD, eF);
        int elevatorPos = elevator.getCurrentPosition();
        double elevatorPIDF = elevatorController.calculate(elevatorPos, elevatorTarget);
//        elevator.setPower(elevatorPIDF);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Left Climb
        leftClimbController.setPIDF(cP, cI, cD, cF);
        int leftClimbPos = leftClimb.getCurrentPosition();
        double leftClimbPIDF = leftClimbController.calculate(leftClimbPos, climbTarget);
//        leftClimb.setPower(leftClimbPIDF);
        leftClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Climb
        rightClimbController.setPIDF(cP, cI, cD, cF);
        int rightClimbPos = rightClimb.getCurrentPosition();
        double rightClimbPIDF = rightClimbController.calculate(rightClimbPos, climbTarget);
//        rightClimb.setPower(rightClimbPIDF);
        rightClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

//---------------------------------------------------------------------------

        //Drive Control

        //Slow Driving
//        if (Intaking || Scoring || Climbing) {
//            LeftStickY = gamepad1.left_stick_y * 0.25;
//            LeftStickX = gamepad1.left_stick_x * 0.25;
//            RX = gamepad1.right_stick_x * 0.3;
//        }

        //Normal Driving
//        else {
            LeftStickY = -gamepad1.left_stick_y;
            LeftStickX = -gamepad1.left_stick_x;
            RX = gamepad1.right_stick_x * 0.8;
//        }

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
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

//---------------------------------------------------------------------------

        //Elevator Control
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(gamepad2.right_stick_y);

        if (gamepad1.a) {
            Scoring = false;

            elevatorTarget = elevatorStowed;
        }

        if (gamepad1.b) {
            Scoring = true;

            elevatorTarget = elevatorLvl1;
        }

        if (gamepad1.y) {
            Scoring = true;
            elevatorTarget = elevatorLvl2;
        }

        if (gamepad1.x) {
            Scoring = true;
            elevatorTarget = elevatorLvl3;
        }

//---------------------------------------------------------------------------

        //Climb Control
        leftClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftClimb.setPower(gamepad2.left_stick_y);
        rightClimb.setPower(gamepad2.left_stick_y);

        if (gamepad2.x //gamepad2.dpad_up && gamepad2.right_bumper
         ) {
            Climbing = true;
            climbTarget = climbDeploy;
        }

        if (gamepad2.b //gamepad2.dpad_down && Climbing
        ) {
            climbTarget = climbQuickUp;
        }

        if (gamepad2.a //gamepad2.dpad_right && Climbing
        ) {
            Climbing = false;
            climbTarget = climbRetract;
        }

//---------------------------------------------------------------------------

        //Intake Control
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.right_bumper) {
            Intaking = true;
            extendIntake.setPosition(intakePickUp);
            intake.setPower(intakeIntaking);
        }

        else {
            Intaking = false;
            extendIntake.setPosition(intakeStowed);
            intake.setPower(intakeHold);
        }

//---------------------------------------------------------------------------

        //Handoff

        wrist.setPosition(gamepad2.left_trigger);

//        //Close Claw
//        if (gamepad2.left_bumper) {
////            openClaw.setPosition(clawClosed);
//        }
//
//        //Open Claw
//        if (gamepad2.right_bumper) {
////            openClaw.setPosition(clawOpen);
//        }
//
//        //Pickup Arm
//        if (gamepad2.a) {
//            leftArm.setPosition(leftArmPickUp);
//            rightArm.setPosition(rightArmPickUp);
//        }
//
//        //Score Arm
//        if (gamepad2.b) {
//            leftArm.setPosition(leftArmScore);
//            rightArm.setPosition(rightArmScore);
//        }
//
//        //Stow Arm
//        if (gamepad2.x) {
//            leftArm.setPosition(leftArmStowed);
//            rightArm.setPosition(rightArmStowed);
//        }

//---------------------------------------------------------------------------

        //Drone Control

//---------------------------------------------------------------------------

        //Update Variables

        //Update previous button states
        previousLTriggerState = currentLTriggerState;
        previousRTriggerState = currentRTriggerState;
        previous2ndLTriggerState = current2ndLTriggerState;
        previous2ndRTriggerState = current2ndRTriggerState;

//---------------------------------------------------------------------------

        //Telemetry Update
        //Drive Information
        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));

        telemetry.addData("Left Arm Pos: ", leftArm.getPosition());
        telemetry.addData("Right Arm Pos", rightArm.getPosition());
        telemetry.addData("Wrist Pos:", wrist.getPosition());
        telemetry.addData("Spin Claw Pos: ", spinClaw.getPosition());
        telemetry.addData("Claw Pos: ", claw.getPosition());

//        telemetry.addData("Left Stick X:", gamepad1.left_stick_x);
//        telemetry.addData("Left Stick Y:", gamepad1.left_stick_y);
//        telemetry.addData("Right Stick X:", gamepad1.right_stick_x);
//        telemetry.addData("Elevator:", elevator.getCurrentPosition());
//        telemetry.addData("Elevator Target:", elevator.getTargetPosition());
//        telemetry.addData("Left Climb:", leftClimb.getCurrentPosition());
//        telemetry.addData("Right Climb:", rightClimb.getCurrentPosition());
//        telemetry.addData("Climb Target:", climbTarget);
//        telemetry.addData("Intaking", Intaking);
//        telemetry.addData("Scoring", Scoring);
//        telemetry.addData("Climbing", Climbing);
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