package org.firstinspires.ftc.teamcode.Season.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class N_Claw extends LinearOpMode {

    boolean ClawOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //Servo Declaration
        Servo Claw = hardwareMap.servo.get("Claw");

        //Initialise Servos
        Claw.setPosition(0.2);

        // Track the previous state of the button
        boolean previousRBumperButtonState = false;

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Check the current state of buttons
            boolean currentRBumperButtonState = gamepad1.right_bumper;

            //Toggle Grab
            // Check if the button is currently pressed and was not pressed in the previous iteration
            if (currentRBumperButtonState && !previousRBumperButtonState) {
                if (ClawOpen) {
                    Claw.setPosition(0);
                    // Claw close
                } else {
                    Claw.setPosition(0.2);
                    // Claw open
                }
                ClawOpen = !ClawOpen; // Toggle the flag
            }

            previousRBumperButtonState = currentRBumperButtonState;
            // Update the previous button state


//            //HOLD TO GRAB - Claw Control
//            if(gamepad2.dpad_right)
//                Claw.setPosition(0.1);
//            else
//                Claw.setPosition(-0.1);

            //Telemetry Update
            telemetry.addData("Claw State", ClawOpen ? "Open" : "Closed");
            telemetry.update();
        }
    }
}
