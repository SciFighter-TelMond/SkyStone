package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *

 */
public class MecRobotDrive {

    /* Public OpMode members. */
    private DcMotor fl_Drive = null;
    private DcMotor fr_Drive = null;
    private DcMotor bl_Drive = null;
    private DcMotor br_Drive = null;
    private Toggle boostState = new Toggle();
    private DcMotor l_roller = null;
    private DcMotor r_roller = null;

    private Servo l_roller_servo = null;
    private Servo r_roller_servo = null;

    private Servo   hooks    = null;
    private Toggle hooksState = new Toggle();

    private DigitalChannel leftBumper = null;
    private DigitalChannel rightBumper = null;
    private DigitalChannel cubeBumper = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MecRobotDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map


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
        hooksState.update(false);

        // get a reference to our digitalTouch object.
        leftBumper = hardwareMap.get(DigitalChannel.class, "left_bumper");
        rightBumper = hardwareMap.get(DigitalChannel.class, "right_bumper");
        cubeBumper = hardwareMap.get(DigitalChannel.class, "cube_bumper");

        leftBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        rightBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.



    }
    public void straight(double target_meter, double speed) {
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

        while((fl_Drive.isBusy() || bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy())  ) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition())<500 || Math.abs(distToTarget)<500) {
                power = 0.5;
            } else {
                power = 1;
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


        }

    }

    public void side(double target_meter, double speed) {
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

        while((fl_Drive.isBusy() || bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()) ) {

        }

    }

    public void stop(){
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double power = 0;
        fl_Drive.setPower(power);
        fr_Drive.setPower(power);
        bl_Drive.setPower(power);
        br_Drive.setPower(power);

        hooks.setPosition(0);
        hooksState.update(false);
    }

    public void hooksDown(){
        hooks.setPosition(1);
        hooksState.update(true);

    }
    public void hooksUp(){
        hooks.setPosition(0);
        hooksState.update(false);

    }
    public boolean getHooksState(){
        return hooksState.getState();
    }

}

