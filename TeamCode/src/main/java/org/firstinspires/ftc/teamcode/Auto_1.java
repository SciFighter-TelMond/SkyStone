package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.text.GetChars;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.ByteArray;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Auto_1", group="Auto")
//@Disabled
public class Auto_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl_Drive = null;
    private DcMotor fr_Drive = null;
    private DcMotor bl_Drive = null;
    private DcMotor br_Drive = null;

    private Servo hooks        = null;
    private Boolean hooksState = false;

    private DigitalChannel leftBumper  = null;
    private DigitalChannel rightBumper = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl_Drive = hardwareMap.get(DcMotor.class, "fl_drive");
        fr_Drive = hardwareMap.get(DcMotor.class, "fr_drive");
        bl_Drive = hardwareMap.get(DcMotor.class, "bl_drive");
        br_Drive = hardwareMap.get(DcMotor.class, "br_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fl_Drive.setDirection(DcMotor.Direction.REVERSE);
        fr_Drive.setDirection(DcMotor.Direction.FORWARD);
        bl_Drive.setDirection(DcMotor.Direction.REVERSE);
        br_Drive.setDirection(DcMotor.Direction.FORWARD);

        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setPosition(0);

        // get a reference to our digitalTouch object.
        leftBumper = hardwareMap.get(DigitalChannel.class, "left_bumper");
        rightBumper = hardwareMap.get(DigitalChannel.class, "right_bumper");

        leftBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        rightBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        move(-1);
        stopMove();
        hooks.setPosition(0.5);
        sleep(1000);
        move(1);
        stopMove();
        hooks.setPosition(0);
        sleep(1000);
        sideMove(1);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Rest in peas: " + runtime.toString());
        telemetry.update();

        sleep(2000);
        stopMove();
    }

    void move(double target_meter) {
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int)(1400 * target_meter);

        fl_Drive.setTargetPosition(ticks);
        fr_Drive.setTargetPosition(ticks);
        bl_Drive.setTargetPosition(ticks);
        br_Drive.setTargetPosition(ticks);

        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 0.5;

        while((fl_Drive.isBusy() /* || bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) && opModeIsActive() ) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition())<500 || Math.abs(distToTarget)<500) {
                power = 0.5;
            } else {
                power = 0.7;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);


            if (leftBumper.getState() == false) {
                fl_Drive.setTargetPosition(fl_Drive.getCurrentPosition());
                bl_Drive.setTargetPosition(bl_Drive.getCurrentPosition());
            }

            if (rightBumper.getState() == false) {
                fr_Drive.setTargetPosition(fr_Drive.getCurrentPosition());
                br_Drive.setTargetPosition(br_Drive.getCurrentPosition());
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Motor Runs to target: %2.2g", target_meter);
            telemetry.addData("Motors", "left (%d), right (%d)", fl_Drive.getCurrentPosition(), fr_Drive.getCurrentPosition());
            telemetry.update();
        }

    }

    void sideMove(double target_meter) {
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int)(1400 * target_meter);

        fl_Drive.setTargetPosition(ticks);
        fr_Drive.setTargetPosition(-ticks);
        bl_Drive.setTargetPosition(-ticks);
        br_Drive.setTargetPosition(ticks);

        double power = 1;
        fl_Drive.setPower(power);
        fr_Drive.setPower(power);
        bl_Drive.setPower(power);
        br_Drive.setPower(power);

        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((fl_Drive.isBusy() || bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()) && opModeIsActive() ) {
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Motor Runs to target: %2.2g", target_meter);
            telemetry.addData("Motors", "left (%d), right (%d)", fl_Drive.getCurrentPosition(), fr_Drive.getCurrentPosition());
            telemetry.update();
        }

    }

    void stopMove(){
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double power = 0;
        fl_Drive.setPower(power);
        fr_Drive.setPower(power);
        bl_Drive.setPower(power);
        br_Drive.setPower(power);
    }
}
