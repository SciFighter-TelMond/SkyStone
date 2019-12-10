package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class DriveClass {

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
    private LinearOpMode opMode = null;
    private HardwareMap hwMap   = null;
    public enum Direction {LEFT, RIGHT, FORWARD, REVERSE};


    /* Constructor */
    public DriveClass(LinearOpMode opMode) { this.opMode=opMode; }

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
        r_roller_servo.setPosition(0);
        l_roller_servo.setPosition(1);// close them

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


    // ==================================================================================================
    public void drive(double straight, double side, double turn, double speedTrigger, double turnTrigger) {

        double speedBoost = speedTrigger * 0.5 + 0.5;
        double turnBoost  = turnTrigger * 0.5 + 0.5;

        double fl_power = (straight + turn * turnBoost + side) * speedBoost;
        double fr_power = (straight - turn * turnBoost - side) * speedBoost;
        double bl_power = (straight + turn * turnBoost - side) * speedBoost;
        double br_power = (straight - turn * turnBoost + side) * speedBoost;

        double m = Math.max(Math.max(fl_power,fr_power), Math.max(bl_power,br_power));
        if (m > 1) {
            fl_power /= m;
            fr_power /= m;
            bl_power /= m;
            br_power /= m;
        }

        boostState.update(speedTrigger>0.7);
        if (boostState.isChanged()) {
            if (boostState.isPressed()) {
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

        opMode.telemetry.addData("Bumper", "left (%b), right (%b)", leftBumper.getState(), rightBumper.getState());

        if (speedBoost<0.7) {
            if (leftBumper.getState() == false) {
                fl_power = Math.max(0, fl_power);
                bl_power = Math.max(0, bl_power);
            }

            if (rightBumper.getState() == false) {
                fr_power = Math.max(0, fr_power);
                br_power = Math.max(0, br_power);
            }
        }

        // Send calculated power to wheels
        fl_Drive.setPower(fl_power);
        fr_Drive.setPower(fr_power);
        bl_Drive.setPower(bl_power);
        br_Drive.setPower(br_power);

    }

    public void straight(double target_meter, Direction direction, double speed, int timeout) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int dir = 1;
        if(direction == Direction.REVERSE)
            dir = -1;
        int ticks = (int)(1400 * target_meter)*dir;

        fl_Drive.setTargetPosition(ticks);
        fr_Drive.setTargetPosition(ticks);
        bl_Drive.setTargetPosition(ticks);
        br_Drive.setTargetPosition(ticks);

        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = drivePower;

        while((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive()  &&  runTime.seconds() < timeout) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition())<200 || Math.abs(distToTarget)<700) {
                power = drivePower/2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);

            if (direction == Direction.REVERSE) {
                if (leftBumper.getState() == false) {
                    fl_Drive.setTargetPosition(fl_Drive.getCurrentPosition());
                    bl_Drive.setTargetPosition(bl_Drive.getCurrentPosition());
                }

                if (rightBumper.getState() == false) {
                    fr_Drive.setTargetPosition(fr_Drive.getCurrentPosition());
                    br_Drive.setTargetPosition(br_Drive.getCurrentPosition());
                }
            }
            opMode.sleep(100);
        }

    }

    public void side(double target_meter, Direction direction, double speed, int timeout) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int dir = 1;
        if(direction == Direction.LEFT)
            dir = -1;
        int ticks = (int)(1400 * target_meter) * dir;

         fl_Drive.setTargetPosition(ticks);
        fr_Drive.setTargetPosition(-ticks);
        bl_Drive.setTargetPosition(-ticks);
        br_Drive.setTargetPosition(ticks);

        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = speed;

        while((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive()  && runTime.seconds() < timeout  ) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition())<200 || Math.abs(distToTarget)<700) {
                power = drivePower/2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);
            opMode.telemetry.addData("fl",fl_Drive.getCurrentPosition() );
            opMode.telemetry.addData("fr",fr_Drive.getCurrentPosition() );
            opMode.telemetry.addData("bl",bl_Drive.getCurrentPosition() );
            opMode.telemetry.addData("br",br_Drive.getCurrentPosition() );
            opMode.telemetry.update();

            opMode.sleep(100);
        }
    }
/*
* rotate
* right rounds > 0, left rounds < 0
* direction LEFT or RIGHT
* speed between 0 and 1
* timeout maximum time in seconds for operation*/
    public void rotate(double rounds, Direction direction, double speed, double timeout) {

        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* 1 round is 5600 ticks
        * to the right posotive
        * to the left negative*/
        int dir = 1;
        if(direction == Direction.LEFT)
            dir = -1;
        int ticks = (int)(5600 * rounds) * dir;



        fl_Drive.setTargetPosition(ticks);
        fr_Drive.setTargetPosition(-ticks);
        bl_Drive.setTargetPosition(ticks);
        br_Drive.setTargetPosition(-ticks);



        fl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = speed;

        while((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) && opMode.opModeIsActive()  && runtime.seconds() < timeout ) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition())<200 || Math.abs(distToTarget)<700) {
                power = drivePower/2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);
            opMode.telemetry.addData("fl",fl_Drive.getCurrentPosition() );
            opMode.telemetry.addData("fr",fr_Drive.getCurrentPosition() );
            opMode.telemetry.addData("bl",bl_Drive.getCurrentPosition() );
            opMode.telemetry.addData("br",br_Drive.getCurrentPosition() );
            opMode.telemetry.update();

            opMode.sleep(100);
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
        rollersStop();

        hooks.setPosition(0);
        hooksState.set(false);
    }

    public void hooksDown(){
        hooks.setPosition(1);
        hooksState.set(true);

    }
    public void hooksUp(){
        hooks.setPosition(0);
        hooksState.set(false);

    }
    public boolean getHooksState(){
        return hooksState.getState();
    }

    public void rollers( boolean open) {
        if (open) {
            r_roller.setPower(1);
            l_roller.setPower(1);
            r_roller_servo.setPosition(1);
            l_roller_servo.setPosition(0);
        } else {
            r_roller_servo.setPosition(0);
            l_roller_servo.setPosition(1);
        }
    }

    public void rollersIn() {
        double rollerPower = 1;
        if (cubeBumper.getState() == false) {
            rollerPower = 0.1;
        }
        r_roller.setPower(rollerPower);
        l_roller.setPower(rollerPower);
    }

    public void rollersOut() {
        r_roller.setPower(-1);
        l_roller.setPower(-1);
    }

    public void rollersStop() {
        r_roller.setPower(0);
        l_roller.setPower(0);
    }
}

