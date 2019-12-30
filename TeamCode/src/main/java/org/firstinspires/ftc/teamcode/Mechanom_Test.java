package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 FTC - SCI-Fighter
 Mechanom Drive 3

 Drive with left right bumpers stops and Hooks.
 */

@TeleOp(name="Mechanom Test", group="Linear Opmode")
@Disabled
public class Mechanom_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fl_Drive = null;
    private DcMotor fr_Drive = null;
    private DcMotor bl_Drive = null;
    private DcMotor br_Drive = null;
    private Toggle boostState = new Toggle();

    private Servo   hooks    = null;
    private Toggle hooksState = new Toggle();

    private DigitalChannel leftBumper = null;
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

        fl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setPosition(0);

        // get a reference to our digitalTouch object.
        leftBumper  = hardwareMap.get(DigitalChannel.class, "left_bumper");
        rightBumper = hardwareMap.get(DigitalChannel.class, "right_bumper");

        leftBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        rightBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double speedTrigger =  gamepad1.right_trigger;
            double turneTrigger =  gamepad1.left_trigger;

            boolean hookBtn     =  gamepad1.right_bumper;

            hooksState.update(hookBtn);
            if(hooksState.isPressed()) {
                if(hooksState.getState())
                    hooks.setPosition(1);
                else
                    hooks.setPosition(0);
            }

            double fl_power =  gamepad1.left_stick_x;
            double fr_power =  gamepad1.right_stick_x;
            double bl_power = -gamepad1.left_stick_y;
            double br_power = -gamepad1.right_stick_y;

            boostState.update(speedTrigger>0.7);
            if (boostState.isChanged()) {
                if (boostState.getState()) {
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

            // Send calculated power to wheels
            fl_Drive.setPower(fl_power);
            fr_Drive.setPower(fr_power);
            bl_Drive.setPower(bl_power);
            br_Drive.setPower(br_power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl_power, fr_power);
            telemetry.addData("Bumper", "left (%b), right (%b)", leftBumper.getState(), rightBumper.getState());
            if (hooksState.getState()) telemetry.addData("Hooks", "ON");
            telemetry.update();
        }
    }
}
