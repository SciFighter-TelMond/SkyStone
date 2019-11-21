/*
 Arm 1 - Simple Drive for the arm with the gamepad2 left and right sticks for Testing Only.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

 */

@TeleOp(name="Arm1", group="Drive")
//@Disabled
public class Arm1 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel startGame0 = null;
    private DigitalChannel startGame1 = null;


     //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        arm0 = hardwareMap.get(DcMotor.class, "arm0");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        startGame0 = hardwareMap.get(DigitalChannel.class, "start_game0");
        startGame1 = hardwareMap.get(DigitalChannel.class, "start_game1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        startGame0.setMode(DigitalChannel.Mode.INPUT);
        startGame1.setMode(DigitalChannel.Mode.INPUT);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


     //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
    }


     //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
    }


     //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        double speed0 = (-1 * gamepad2.right_stick_y) /3;
        double speed1 = (-1 * gamepad2.left_stick_y) / 4;

        if (startGame0.getState() == false){
            speed0 = Math.max(0,speed0);
        }
        if (startGame1.getState() == false){
            speed1 = Math.min(0,speed1);
        }
        arm0.setPower(speed0);
        arm1.setPower(speed1);




        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arms", "arm0 (%d), arm1 (%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());

        telemetry.update();

    }


     //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }

    //int ticksToMeters(){}

}
