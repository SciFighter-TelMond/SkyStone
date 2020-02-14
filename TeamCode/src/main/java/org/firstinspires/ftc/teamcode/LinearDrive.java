package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * FTC - SCI-Fighters
 * Mechanom Drive 3
 * <p>
 * Drive with left right bumpers stops and Hooks.
 */

@TeleOp(name = "Linear Drive", group = "SciFighters")
//@Disabled
public class LinearDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Toggle hooksState = new Toggle();
    private Toggle rollersRunIn = new Toggle();
    private Toggle rollersRunOut = new Toggle();

    private Toggle clamps = new Toggle();
    private Toggle clamps_rotate = new Toggle();

    private Toggle armDoHome = new Toggle();
    private Toggle armDoPick = new Toggle();
    private Toggle armDoBuild = new Toggle();

    private Toggle armFloorUp = new Toggle();
    private Toggle armFloorDown = new Toggle();

    private Toggle stoneBumperToggle = new Toggle();

    private Toggle capStone = new Toggle();

    private DriveClass drive = new DriveClass(this,true);
    private ArmClass arm = new ArmClass(this, drive);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);
        arm.init(hardwareMap);

        drive.setCapstone(false);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        arm.begin();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // drive.isRed();
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double straight = -gamepad1.right_stick_y;
            double side = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;
            double speedTrigger = gamepad1.right_trigger;
            double turnTrigger = gamepad1.left_trigger;
            boolean slowLeft = gamepad1.dpad_left;
            boolean slowRight = gamepad1.dpad_right;

            if (slowLeft) {
                side = -0.2 * 2;
            }
            if (slowRight) {
                side = 0.2 * 2;
            }
            drive.drive(straight, side, turn, speedTrigger, turnTrigger);

            //==========================================
            //CapStone
            //==========================================
            capStone.update(gamepad1.x);
            if (capStone.isClicked()) {
                drive.setCapstone(capStone.getState());
            }

            // =========================================
            // Hooks Control
            // =========================================
            hooksState.update(gamepad1.right_bumper);
            if (hooksState.isPressed()) {
                if (hooksState.getState())
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
            if (gamepad2.x & gamepad2.dpad_left) {
                arm.reset();
            }

            armDoHome.update(gamepad2.x && gamepad2.dpad_down);
            armDoPick.update(gamepad2.y && gamepad2.dpad_left);
            armDoBuild.update(gamepad2.y && gamepad2.dpad_right);
            armFloorUp.update(gamepad2.y && gamepad2.dpad_up);
            armFloorDown.update(gamepad2.y && gamepad2.dpad_down);

            if (armDoHome.isClicked()) {
                arm.pleaseDo(ArmClass.Mode.HOME);
            }

            if (armDoPick.isClicked()) {
                arm.pleaseDo(ArmClass.Mode.PICK);
            }

            if (armDoBuild.isClicked()) {
                arm.pleaseDo(ArmClass.Mode.BUILD);
            }

            if (armFloorUp.isClicked()) {
                arm.floorPlus();
                arm.pleaseDo(ArmClass.Mode.BUILD);
            }
            if (armFloorDown.isClicked()) {
                arm.floorMinus();
                arm.pleaseDo(ArmClass.Mode.BUILD);
            }


            clamps.update(gamepad2.left_bumper);
            if (clamps.isChanged())
                arm.openClamps(clamps.getState());

            clamps_rotate.update(gamepad2.right_bumper);
            if (clamps_rotate.isChanged())
                arm.rotateClamps(clamps_rotate.getState());

            arm.setBoost(gamepad2.right_trigger + gamepad2.left_trigger);

            // =========================================
            // Rollers Control
            // =========================================
            boolean rollerBtl = gamepad1.left_bumper || (gamepad2.x && gamepad2.dpad_up);
            drive.rollerServoState.update(rollerBtl);
            if (drive.rollerServoState.isChanged())
                drive.rollers(drive.rollerServoState.getState());

            stoneBumperToggle.update(drive.getStoneBumperState());
            rollersRunIn.update(gamepad1.dpad_down);
            rollersRunOut.update(gamepad1.dpad_up);
            if (rollersRunOut.isPressed()) {
                drive.rollersRunOut();
            } else {
                if (rollersRunOut.isChanged() || rollersRunIn.isChanged() || stoneBumperToggle.isChanged()) {
                    if (rollersRunIn.getState()) {
                        drive.rollersRunIn();
                    } else {
                        drive.rollersStop();
                    }
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl_power, fr_power);
            telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm.getArm0Pos(), arm.getArm1Pos());
            // telemetry.addData("Arms Switch", "Arm0:(%b), Arm1:(%b)", arm.getArm0Zero(), arm.getArm1ZeroA());

            //if (hooksState.getState()) telemetry.addData("Hooks", "ON");
            telemetry.update();
        } // end of if (opModeisActive)
        arm.end();
        drive.end();
    } // end of runOpMode()

    void specialLog(String txt){
        RobotLog.d("================================");
        RobotLog.d(txt);
        RobotLog.d("================================");
    }
}
