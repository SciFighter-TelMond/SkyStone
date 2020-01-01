package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class DriveClass {

    /* Public OpMode members. */
    volatile private DcMotor fl_Drive = null;
    volatile private DcMotor fr_Drive = null;
    volatile private DcMotor bl_Drive = null;
    volatile private DcMotor br_Drive = null;
    volatile private DcMotor l_roller = null;
    volatile private DcMotor r_roller = null;

    volatile private Servo l_roller_servo = null;
    volatile private Servo r_roller_servo = null;

    volatile private Toggle boostState = new Toggle();
    volatile private Toggle hooksState = new Toggle();
    volatile private Servo hooks = null;
    volatile private Servo capstone = null;

    volatile private DigitalChannel leftBumper = null;
    volatile private DigitalChannel rightBumper = null;
    volatile private DigitalChannel stoneBumper = null;

    private ColorSensor sensorColorRight;
    private DistanceSensor sensorDistanceRight;
    private ColorSensor sensorColorLeft;
    private DistanceSensor sensorDistanceLeft;
    private ColorSensor sensorColorDown;

    /* local OpMode members. */
    volatile private LinearOpMode opMode = null;
    volatile private HardwareMap hwMap = null;

    public enum Direction {LEFT, RIGHT, FORWARD, REVERSE};

    private boolean useBrake;

    /* Constructor */
    public DriveClass(LinearOpMode opMode,boolean useBrake) {
        this.opMode = opMode;
        this.useBrake = useBrake;
    }
    public DriveClass(LinearOpMode opMode) {
        this.opMode = opMode;
        this.useBrake = true;
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

        if(useBrake) {
            fl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        l_roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r_roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        l_roller_servo = hardwareMap.get(Servo.class, "left_roller_servo");
        r_roller_servo = hardwareMap.get(Servo.class, "right_roller_servo");
        r_roller_servo.setPosition(0);
        l_roller_servo.setPosition(1);// close them

        hooks = hardwareMap.get(Servo.class, "hooks");
        hooks.setPosition(0);
        hooksState.update(false);

        capstone = hardwareMap.get(Servo.class, "capstone");


        // get a reference to our digitalTouch object.
        leftBumper = hardwareMap.get(DigitalChannel.class, "left_bumper");
        rightBumper = hardwareMap.get(DigitalChannel.class, "right_bumper");
        stoneBumper = hardwareMap.get(DigitalChannel.class, "cube_bumper");

        leftBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        rightBumper.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

        sensorColorRight = hardwareMap.get(ColorSensor.class, "color_right");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "color_right");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "color_left");
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "color_left");
        sensorColorDown = hardwareMap.get(ColorSensor.class, "color_down");
    }

    public boolean getStoneBumperState() {
        return stoneBumper.getState();
    }

    // ==================================================================================================
    public void drive(double straight, double side, double turn, double speedTrigger, double turnTrigger) {

        double speedBoost = speedTrigger * 0.5 + 0.5;
        double turnBoost = turnTrigger * 0.5 + 0.5;

        double fl_power = (straight + side) * speedBoost + turn * turnBoost;
        double fr_power = (straight - side) * speedBoost - turn * turnBoost;
        double bl_power = (straight - side) * speedBoost + turn * turnBoost;
        double br_power = (straight + side) * speedBoost - turn * turnBoost;

        double m = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));
        if (m > 1) {
            fl_power /= m;
            fr_power /= m;
            bl_power /= m;
            br_power /= m;
        }

        boolean oldMode = fl_Drive.getMode() == DcMotor.RunMode.RUN_TO_POSITION;
        boostState.update(speedTrigger > 0.7 || turnTrigger > 0.85);
        if (boostState.isChanged() || oldMode) {
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

        //opMode.telemetry.addData("Bumper", "left (%b), right (%b)", leftBumper.getState(), rightBumper.getState());

        if (speedBoost < 0.7) {
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
        if (direction == Direction.REVERSE)
            dir = -1;
        int ticks = (int) (1400 * target_meter) * dir;

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

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive() && runTime.seconds() < timeout) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition()) < 200 || Math.abs(distToTarget) < 700) {
                power = drivePower / 2;
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
            opMode.sleep(20);
        }

    }

    public void straight_ignoreBumper(double target_meter, Direction direction, double speed, int timeout) {
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
        if (direction == Direction.LEFT)
            dir = -1;
        int ticks = (int) (1400 * target_meter) * dir;

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

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive() && runTime.seconds() < timeout) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition()) < 200 || Math.abs(distToTarget) < 700) {
                power = drivePower / 2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);
            opMode.telemetry.addData("fl", fl_Drive.getCurrentPosition());
            opMode.telemetry.addData("fr", fr_Drive.getCurrentPosition());
            opMode.telemetry.addData("bl", bl_Drive.getCurrentPosition());
            opMode.telemetry.addData("br", br_Drive.getCurrentPosition());
            opMode.telemetry.update();

            opMode.sleep(20);
        }
    }

    /*
     * rotate
     * right rounds > 0, left rounds < 0
     * direction LEFT or RIGHT
     * speed between 0 and 1
     * timeout maximum time in seconds for operation*/
    public void rotate(double rounds, Direction direction, double speed, double timeout) {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* 1 round is 5600 ticks
         * to the right posotive
         * to the left negative*/
        int dir = 1;
        if (direction == Direction.LEFT)
            dir = -1;
        int ticks = (int) (5600 * rounds) * dir;

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

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) && opMode.opModeIsActive() && runtime.seconds() < timeout) {

            int distToTarget = ticks - fr_Drive.getCurrentPosition();

            if (Math.abs(fr_Drive.getCurrentPosition()) < 200 || Math.abs(distToTarget) < 700) {
                power = drivePower / 2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);
            opMode.telemetry.addData("fl", fl_Drive.getCurrentPosition());
            opMode.telemetry.addData("fr", fr_Drive.getCurrentPosition());
            opMode.telemetry.addData("bl", bl_Drive.getCurrentPosition());
            opMode.telemetry.addData("br", br_Drive.getCurrentPosition());
            opMode.telemetry.update();

            opMode.sleep(20);
        }
    }

    public void stop() {
        fl_Drive.setPower(0);
        fr_Drive.setPower(0);
        bl_Drive.setPower(0);
        br_Drive.setPower(0);

        if (fl_Drive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            fl_Drive.setTargetPosition(fl_Drive.getCurrentPosition());
            fr_Drive.setTargetPosition(fr_Drive.getCurrentPosition());
            bl_Drive.setTargetPosition(bl_Drive.getCurrentPosition());
            br_Drive.setTargetPosition(br_Drive.getCurrentPosition());
        }
    }

    public void end() {
        fl_Drive.setPower(0);
        fr_Drive.setPower(0);
        bl_Drive.setPower(0);
        br_Drive.setPower(0);

        fl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rollersStop();
        hooks.setPosition(0);
        hooksState.set(false);
    }

    public void hooksDown() {
        hooks.setPosition(1);
        hooksState.set(true);

    }

    public void hooksUp() {
        hooks.setPosition(0);
        hooksState.set(false);

    }

    public boolean getHooksState() {
        return hooksState.getState();
    }

    public void rollers(boolean open) {
        if (open) {
            // r_roller.setPower(1);
            // l_roller.setPower(1);
            r_roller_servo.setPosition(1);
            l_roller_servo.setPosition(0);
        } else {
            r_roller_servo.setPosition(0);
            l_roller_servo.setPosition(1);
        }
    }

    public double getRollersPower() {
        return r_roller.getPower();
    }

    public void rollersRunIn() {
        double rollerPower = 1;
        if (stoneBumper.getState() == false) {
            rollerPower = 0.05;
        }
        r_roller.setPower(rollerPower);
        l_roller.setPower(rollerPower);
    }

    public void rollersRunOut() {
        r_roller.setPower(-1);
        l_roller.setPower(-1);
    }

    public void rollersStop() {
        r_roller.setPower(0);
        l_roller.setPower(0);
    }

    public ColorSensor getSensorColorDown() {
        return sensorColorDown;
    }

    public ColorSensor getSensorColorRight() {
        return sensorColorRight;
    }

    public ColorSensor getSensorColorLeft() {
        return sensorColorLeft;
    }

    public double getSensorDistanceRight() {
        return sensorDistanceRight.getDistance(DistanceUnit.CM);
    }

    public double getSensorDistanceLeft() {
        return sensorDistanceLeft.getDistance(DistanceUnit.CM);
    }

    public boolean isSkyStoneRight() {
        return isSkyStone(sensorColorRight);
    }
    public boolean isSkyStoneLeft(){
        return isSkyStone(sensorColorLeft);
    }

    boolean isSkyStone(ColorSensor sensor) {
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double a = sensor.alpha();

        float hsvValues[] = {0F, 0F, 0F};
        final int SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensor.red() * SCALE_FACTOR),
                (int) (sensor.green() * SCALE_FACTOR),
                (int) (sensor.blue() * SCALE_FACTOR),
                hsvValues);

        float hue = hsvValues[0];
        boolean skystone = hue > 110 && a > 600;
        opMode.telemetry.addData("Sky:","%b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", skystone,hue,r,g,b,a );
        return skystone;
    }

    public void setCapstone(boolean open) {
        if (open == true) {
            capstone.setPosition(0);
        }

        if (open == false) {
            capstone.setPosition(1);
        }

    }

    public  boolean isRed() {
        ColorSensor sensor = sensorColorDown;
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double a = sensor.alpha();

        float hsvValues[] = {0F, 0F, 0F};
        final int SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensor.red() * SCALE_FACTOR),
                (int) (sensor.green() * SCALE_FACTOR),
                (int) (sensor.blue() * SCALE_FACTOR),
                hsvValues);

        float hue = hsvValues[0];
        boolean red = Math.abs(hue) < 80 ;
        opMode.telemetry.addData("Red:","%b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", red,hue,r,g,b,a );
        return red;
    }

    public  boolean isBlue() {
        ColorSensor sensor = sensorColorDown;
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double a = sensor.alpha();

        float hsvValues[] = {0F, 0F, 0F};
        final int SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensor.red() * SCALE_FACTOR),
                (int) (sensor.green() * SCALE_FACTOR),
                (int) (sensor.blue() * SCALE_FACTOR),
                hsvValues);

        float hue = hsvValues[0];
        boolean blue = hue > 220 && hue < 260;
        opMode.telemetry.addData("Red:","%b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", blue,hue,r,g,b,a );
        return blue;
    }
}


