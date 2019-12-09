package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
    FTC - SCI-Fighter
    Mechanom Drive 3

    Drive with left right bumpers stops and Hooks.
 */

@TeleOp(name="Linear Drive", group="Linear Opmode")
//@Disabled
public class LinearDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Toggle hooksState       = new Toggle();
    private Toggle rollerState      = new Toggle();
    private Toggle rollerServoState = new Toggle();
    private Toggle clamps           = new Toggle();
    private Toggle clamps_rotate    = new Toggle();

    private DriveClass drive = new DriveClass(this);
    private ArmClass   arm   = new ArmClass(this, drive);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);
        arm.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        arm.begin();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double straight     = -gamepad1.right_stick_y;
            double side         =  gamepad1.right_stick_x;
            double turn         =  gamepad1.left_stick_x;
            double speedTrigger =  gamepad1.right_trigger;
            // double turneTrigger =  gamepad1.left_trigger;

            drive.drive(straight, side, turn, speedTrigger, speedTrigger);

            // =========================================
            // Hooks Control
            // =========================================
            hooksState.update(gamepad1.right_bumper);
            if(hooksState.isPressed()) {
                if(hooksState.getState())
                    drive.hooksDown();
                else
                    drive.hooksUp();
            }

            // =========================================
            // Arm Control
            // =========================================
            arm.moveArm0(-gamepad2.left_stick_y);
            arm.moveArm1(-gamepad2.right_stick_y);
            if (gamepad2.b || gamepad1.b) {
                arm.end();
                // arm.resumePower();
            }
            if (gamepad2.a) {
                arm.resumePower();
            }
            if (gamepad2.x) {
                arm.reset();
            }
            if (gamepad2.y) {
                arm.linearDo(ArmClass.Mode.PICK);
            }

            clamps.update(gamepad2.left_bumper);
            if (clamps.isChanged())
                arm.clamp(clamps.getState());

            clamps_rotate.update(gamepad2.right_bumper);
            if (clamps_rotate.isChanged())
                arm.rotateClamp(clamps_rotate.getState());

            arm.setBoost(gamepad2.right_trigger + gamepad2.left_trigger);

            // =========================================
            // Rollers Control
            // =========================================
            rollerServoState.update(gamepad1.left_bumper);
            drive.rollers(rollerServoState.getState());

            rollerState.update(gamepad1.dpad_down);
            if (gamepad1.dpad_up) {
                drive.rollersOut();
            } else {
                if (rollerState.getState()) {
                    drive.rollersIn();
                } else {
                    drive.rollersStop();
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl_power, fr_power);
            telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm.getArm0Pos(), arm.getArm1Pos());
            telemetry.addData("Arms Switch", "Arm0:(%b), Arm1 a:(%b), b:(%b)", arm.getArm0Zero(), arm.getArm1ZeroA(), arm.getArm1ZeroB());

            if (hooksState.getState()) telemetry.addData("Hooks", "ON");
            telemetry.update();
        }
    }
}
