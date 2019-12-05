/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Iterative Drive 2", group = "Iterative Opmode")
//@Disabled
public class IterativeDrive2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fl_Drive = null;
    private DcMotor fr_Drive = null;
    private DcMotor bl_Drive = null;
    private DcMotor br_Drive = null;

    private DcMotor l_roller = null;
    private DcMotor r_roller = null;

    private Servo l_roller_servo = null;
    private Servo r_roller_servo = null;

    private Servo hooks = null;
    private Toggle hooksState = new Toggle();
    private Toggle speedState = new Toggle();
    private Toggle rollerState = new Toggle();

    private Toggle rollerServoState = new Toggle();
    private DigitalChannel cubeBumper = null;

    private DigitalChannel leftBumper = null;
    private DigitalChannel rightBumper = null;

    private ArmClass arm = new ArmClass();

    private AndroidTextToSpeech tts;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        fl_Drive = hardwareMap.get(DcMotor.class, "fl_drive");
        fr_Drive = hardwareMap.get(DcMotor.class, "fr_drive");
        bl_Drive = hardwareMap.get(DcMotor.class, "bl_drive");
        br_Drive = hardwareMap.get(DcMotor.class, "br_drive");
        l_roller = hardwareMap.get(DcMotor.class, "left_roller");
        r_roller = hardwareMap.get(DcMotor.class, "right_roller");

        // Set Motor directions for driving forward
        fl_Drive.setDirection(DcMotor.Direction.REVERSE);
        fr_Drive.setDirection(DcMotor.Direction.FORWARD);
        bl_Drive.setDirection(DcMotor.Direction.REVERSE);
        br_Drive.setDirection(DcMotor.Direction.FORWARD);

        l_roller.setDirection(DcMotor.Direction.REVERSE);
        r_roller.setDirection(DcMotor.Direction.FORWARD);


        // Set Driving mode for speed control using the encoders.
        fl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        l_roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r_roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        l_roller_servo = hardwareMap.get(Servo.class, "left_roller_servo");
        r_roller_servo = hardwareMap.get(Servo.class, "right_roller_servo");

        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setPosition(0);

        // get a reference to our digitalTouch object.
        leftBumper  = hardwareMap.get(DigitalChannel.class, "left_bumper");
        rightBumper = hardwareMap.get(DigitalChannel.class, "right_bumper");
        cubeBumper  = hardwareMap.get(DigitalChannel.class, "cube_bumper");

        leftBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        rightBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

        tts = new AndroidTextToSpeech();


        arm.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        tts.initialize();
        arm.begin();
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double straight = -gamepad1.right_stick_y;
        double side = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x * 0.7;
        double speedTrigger = gamepad1.right_trigger;

        // =========================================
        // Hooks Control
        // =========================================
        hooksState.update(gamepad1.right_bumper);

        if (hooksState.isPressed()) {
            if (hooksState.getState())
                hooks.setPosition(1);
            else
                hooks.setPosition(0);
        }

        // =========================================
        // Arm Control
        // =========================================
        arm.moveArm0(-gamepad2.left_stick_y);
        arm.moveArm1(-gamepad2.right_stick_y);
        if (gamepad2.b || gamepad1.b) {
            arm.end();
            arm.resumePower();
        }
        if (gamepad2.a) {
            arm.resumePower();
        }
        if (gamepad2.x) {
            arm.reset();
        }
        if (gamepad2.y) {
            arm.pleaseDo(ArmClass.Mode.PICK);
        }

        if (gamepad2.dpad_right){
            tts.speak("Ubuntu");
        }
        if (gamepad2.dpad_up){
            tts.speak("to to to to to");
        }
        if (gamepad2.dpad_down){
            tts.speak("hello my name is Mantis");
        }

        if (gamepad2.left_bumper) {
            telemetry.addData("left bumper true: clamp open", true);
            telemetry.update();
            arm.clamp(true);
        }
        if (gamepad2.right_bumper) {
            telemetry.addData("left bumper false: clamp close", false);
            telemetry.update();
            arm.clamp(false);
        }

        if (gamepad2.left_trigger > 0.5) {
            arm.rotateClamp(true);
        }
        if (gamepad2.right_trigger > 0.5) {
            arm.rotateClamp(false);
        }

        // =========================================
        // Rollers Control
        // =========================================
        rollerServoState.update(gamepad1.left_bumper);
        if (rollerServoState.getState()) {
            r_roller_servo.setPosition(0);
            l_roller_servo.setPosition(1);
        } else {
            r_roller_servo.setPosition(1);
            l_roller_servo.setPosition(0);
        }

        rollerState.update(gamepad1.dpad_down);
        double rollerPower = 1;
        if ((gamepad1.dpad_up == false) && (cubeBumper.getState() == false)) {
            rollerPower = 0.1;
        } else {
            rollerPower = 1;
        }

        if (gamepad1.dpad_up) {
            r_roller.setPower(-rollerPower);
            l_roller.setPower(-rollerPower);
        } else {

            if (rollerState.getState()) {
                r_roller.setPower(rollerPower);
                l_roller.setPower(rollerPower);
            } else {
                r_roller.setPower(0);
                l_roller.setPower(0);
            }
        }

        // =========================================
        // Mechanom Drive with Speed Boost.
        // =========================================
        speedState.update(speedTrigger > 0.7);

        if (speedState.isChanged()) {
            if (speedState.isPressed()) {

                fl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                fr_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bl_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                br_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                fl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fr_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                br_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        double speedBoost = speedTrigger * 0.5 + 0.5;

        double fl_power = (straight + turn + side) * speedBoost;
        double fr_power = (straight - turn - side) * speedBoost;
        double bl_power = (straight + turn - side) * speedBoost;
        double br_power = (straight - turn + side) * speedBoost;

        double m = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));
        if (m > 1) {
            fl_power /= m;
            fr_power /= m;
            bl_power /= m;
            br_power /= m;
        }

        if (leftBumper.getState() == false) {
            fl_power = Math.max(0, fl_power);
            bl_power = Math.max(0, bl_power);
        }

        if (rightBumper.getState() == false) {
            fr_power = Math.max(0, fr_power);
            br_power = Math.max(0, br_power);
        }

        // Send calculated power to wheels
        fl_Drive.setPower(fl_power);
        fr_Drive.setPower(fr_power);
        bl_Drive.setPower(bl_power);
        br_Drive.setPower(br_power);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl_power, fr_power);
        telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm.getArm0Pos(), arm.getArm1Pos());
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        fl_Drive.setPower(0);
        fr_Drive.setPower(0);
        bl_Drive.setPower(0);
        br_Drive.setPower(0);
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.end(); // stop the arm
    }
}
