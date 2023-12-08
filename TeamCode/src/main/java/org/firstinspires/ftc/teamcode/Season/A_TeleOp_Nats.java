package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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
public class A_TeleOp_Nats extends OpMode {

//---------------------------------------------------------------------------

    //Motor Set Points
    int LiftSetPtIntake = Z_Global_Variables.LiftSetPtIntake;
    int LiftSetPtLvl1 = Z_Global_Variables.LiftSetPtLvl1;
    int LiftSetPtLvl2 = Z_Global_Variables.LiftSetPtLvl2;
    int LiftSetPtLvl3 = Z_Global_Variables.LiftSetPtLvl3;
    int LiftSetPtLvl4 = Z_Global_Variables.LiftSetPtLvl4;

    int ClimbSetPtUp = Z_Global_Variables.ClimbSetPtUp;
    int ClimbSetPtDown = Z_Global_Variables.ClimbSetPtDown;

    //Servo Set Points
    double WristSetPtIn = Z_Global_Variables.WristSetPtIn;
    double WristSetPtOut = Z_Global_Variables.WristSetPtOut;
    double WristSetPtScore = Z_Global_Variables.WristSetPtScore;

    double ClawSetPtClosed = Z_Global_Variables.ClawSetPtClosed;
    double ClawSetPtOpen = Z_Global_Variables.ClawSetPtOpen;
    double ClawSetPtSingleSmall = Z_Global_Variables.ClawSetPtSingleSmall;

    double DroneSetPtClosed = Z_Global_Variables.DroneSetPtClosed;
    double DroneSetPtOpen = Z_Global_Variables.DroneSetPtOpen;

    double HookSetPtClosed = Z_Global_Variables.HookSetPtClosed;
    double HookSetPtOpen = Z_Global_Variables.HookSetPtOpen;

    //Auto Pick up sleeps
    int WristSleepDown = Z_Global_Variables.WristSleepDown;
    int WristSleepUp = Z_Global_Variables.WristSleepUp;
    int WristSleepDownSmall = Z_Global_Variables.WristSleepDownSmall;
    int WristSleepUpSmall = Z_Global_Variables.WristSleepUpSmall;
    int WristSleepBack = Z_Global_Variables.WristSleepBack;

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

    //Trigger state initialization
    public double previousRTriggerState = 0;
    public double previousLTriggerState = 0;

    //Lift Reset timer initialization
    public long lastResetTime = 0;

//---------------------------------------------------------------------------

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public DcMotor Lift;
    public DcMotor Climb;

    public Servo Wrist;
    public Servo Claw;
    public Servo Drone;
    public Servo Hook;

//---------------------------------------------------------------------------

    @Override
    public void init() {
        //Code to run ONCE when the driver hits INIT

//---------------------------------------------------------------------------

        //Motor Declaration
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Leftfront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Leftback");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Rightfront");
        backRightMotor = hardwareMap.get(DcMotor.class, "Rightback");

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Climb = hardwareMap.get(DcMotor.class, "Climb");

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

        //Servo Declaration
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Hook = hardwareMap.get(Servo.class, "Hook");

        //Initialise Servos
        Claw.setPosition(ClawSetPtClosed);
        Wrist.setPosition(WristSetPtIn);
        Drone.setPosition(DroneSetPtClosed);
        Hook.setPosition(HookSetPtClosed);

        //Initialise PIDF Loops
        Climb_target = ClimbSetPtDown;
        Lift_target = LiftSetPtIntake;

//---------------------------------------------------------------------------

        //Send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

//---------------------------------------------------------------------------

    @Override
    public void init_loop() {
        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    }

//---------------------------------------------------------------------------

    @Override
    public void start() {
        //Code to run ONCE when the driver hits PLAY

//        //Send elevator down to reset the end pt
//        Lift_target = -10;
    }

//---------------------------------------------------------------------------

    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        //Track Previous State of Buttons
        //2 Pixel Control
        double currentLTriggerState = gamepad1.left_trigger;
        //1 Pixel Control
        double currentRTriggerState = gamepad1.right_trigger;

        //Initialise Servo State
        boolean Intaking = false;

        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

//---------------------------------------------------------------------------

        //Drive Control

        //Slow Driving
        if (gamepad1.left_bumper) {
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

        //Lift Control

        long currentTime = System.currentTimeMillis();
        //Limit Switch Encoder Reset IF TIME HAS PASSED
        if ((!LiftLimitSwitch.getState()) && (Lift.getCurrentPosition() != 0
                && currentTime - lastResetTime >= 1000)) {
            // Limit switch is pressed, reset the motor encoder
            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift_target = 0;
            lastResetTime = currentTime;
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
            //Reduce double clicks
            sleep(500);
        }

        //Shoot Drone
        else if (gamepad1.dpad_right && Climb.getCurrentPosition() < -100 &&
                gamepad1.right_bumper) {
            //Shoot drone
            Drone.setPosition(DroneSetPtOpen);
        }

        //Shoot drone and reset
        if (gamepad1.dpad_up && gamepad1.right_bumper) {
            //Drone is shut, Open Drone
            if (Drone.getPosition() < 0.51) {
                Drone.setPosition(DroneSetPtOpen);
                sleep(500);
            }
            //Drone is open, Shut Drone
            if (Drone.getPosition() > 0.58) {
                Drone.setPosition(DroneSetPtClosed);
                sleep(500);
            }
        }

//---------------------------------------------------------------------------

        //2 Pixel Toggle

        //If Lift in scoring, let go and retract wrist
        if (Lift.getCurrentPosition() >= 100 && gamepad1.right_trigger > 0.1) {
            //retract wrist & Close Claw
            Claw.setPosition(ClawSetPtOpen);
            sleep(WristSleepBack);
            Wrist.setPosition(WristSetPtIn);
            //Lift to position
            Lift_target = LiftSetPtIntake;
        }

        //If lift in intake, start intake sequence
        else if (Lift.getCurrentPosition() < 100) {
            //Check if button has been pressed
            if (currentRTriggerState > 0 && previousRTriggerState <= 0 && !Intaking) {
                Intaking = true;
                //Reach out and grab pixel
                //Claw open
                Claw.setPosition(ClawSetPtOpen);
                //Wrist out
                Wrist.setPosition(WristSetPtOut);
                //Wait
                sleep(WristSleepDown);
                //Grab Pixel
                Claw.setPosition(ClawSetPtClosed);
                //Wait
                sleep(WristSleepUp);
            }

            else if (currentRTriggerState <= 0 && previousRTriggerState > 0) {
                //Pull pixel into robot
                //Wrist in
                Wrist.setPosition(WristSetPtIn);
                Intaking = false;
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
            //Lift to position
            Lift_target = LiftSetPtIntake;
        }

        //If Lift in intake, enable intake sequence
        else if (Lift.getCurrentPosition() < 100) {
            //Check if the button has been pressed
            if (currentLTriggerState > 0 && previousLTriggerState <= 0) {
                //Stop double deployment
                if (!Intaking) {
                    Intaking = true;
                    //Reach out and grab pixel
                    //Claw open
                    Claw.setPosition(ClawSetPtOpen);
                    //Wrist out
                    Wrist.setPosition(WristSetPtOut);
                    //Wait
                    sleep(WristSleepDownSmall);
                    //Grab Pixel
                    Claw.setPosition(ClawSetPtSingleSmall);
                    //Wait
                    sleep(WristSleepUpSmall);
                }
            }

            //Check if the button has been released
            else if (currentLTriggerState <= 0 && previousLTriggerState > 0) {
                //Pull pixel into robot
                //Wrist in
                Wrist.setPosition(WristSetPtIn);
                Intaking = false;
            }
        }

//---------------------------------------------------------------------------

        //Update previous button states

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
        telemetry.addData("8. claw", Claw.getPosition());
        //Wrist Information
        telemetry.addData("7.Wrist State", Intaking? "Out" : "In");
        telemetry.addData("9. wrist", Wrist.getPosition());
        //Drone Information
        telemetry.addData("10. Back Button", gamepad1.back);
        telemetry.addData("11. Drone", Drone.getPosition());
        //Update
        telemetry.update();
    }

        @Override
    public void stop() {
        //Code to run ONCE after the driver hits STOP
    }

    private void sleep (int t) {
        try {
            Thread.sleep(t);
        } catch(InterruptedException e){
            throw new RuntimeException(e);
        }
    }

    }

