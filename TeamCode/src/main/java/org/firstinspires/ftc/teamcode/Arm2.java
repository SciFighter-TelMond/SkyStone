/*
 Arm 1 - Simple Drive for the arm with the gamepad2 left and right sticks for Testing Only.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Arm2", group = "Drive")
//@Disabled
public class Arm2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ArmClass arm = new ArmClass();



    @Override
    public void init() {
        arm.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        arm.begin();
        runtime.reset();
    }

    @Override
    public void loop() {
        arm.moveArm0(-1 * gamepad2.right_stick_y);
        arm.moveArm1(-1 * gamepad2.left_stick_y);
        if (gamepad2.b){
            arm.reset();
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arms", "arm0 (%d), arm1 (%d)", arm.getArm0Pos(), arm.getArm1Pos());
        telemetry.update();
    }


    @Override
    public void stop() {}


}
