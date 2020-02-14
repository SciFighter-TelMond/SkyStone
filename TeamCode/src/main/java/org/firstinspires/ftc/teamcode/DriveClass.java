package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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
    volatile private DcMotorEx fl_Drive = null;
    volatile private DcMotorEx fr_Drive = null;
    volatile private DcMotorEx bl_Drive = null;
    volatile private DcMotorEx br_Drive = null;
    volatile private DcMotorEx l_roller = null;
    volatile private DcMotorEx r_roller = null;

    volatile private Servo l_roller_servo = null;
    volatile private Servo r_roller_servo = null;
    volatile public Toggle rollerServoState = new Toggle();

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


    public enum Direction { LEFT, RIGHT, FORWARD, REVERSE }

    public enum Location { LEFT, RIGHT }


    private boolean useBrake;

    /* Constructor */
    public DriveClass(LinearOpMode opMode, boolean useBrake) {
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

        fl_Drive = hardwareMap.get(DcMotorEx.class, "fl_drive");
        fr_Drive = hardwareMap.get(DcMotorEx.class, "fr_drive");
        bl_Drive = hardwareMap.get(DcMotorEx.class, "bl_drive");
        br_Drive = hardwareMap.get(DcMotorEx.class, "br_drive");

        l_roller = hardwareMap.get(DcMotorEx.class, "left_roller");
        r_roller = hardwareMap.get(DcMotorEx.class, "right_roller");

        // Set Motor directions for driving forward
        fl_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        fr_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        bl_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        br_Drive.setDirection(DcMotorEx.Direction.FORWARD);

        l_roller.setDirection(DcMotorEx.Direction.REVERSE);
        r_roller.setDirection(DcMotorEx.Direction.FORWARD);


        // Set Driving mode for speed control using the encoders.
        fl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fl_Drive.setTargetPositionTolerance(15);
        fr_Drive.setTargetPositionTolerance(15);
        bl_Drive.setTargetPositionTolerance(15);
        br_Drive.setTargetPositionTolerance(15);

        PIDFCoefficients pidf = fr_Drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        pidf.p = 5;
//        pidf.i = 3;
//        pidf.d = 0.1;
////        pidf.f = 14;
//        fr_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
////        fr_Drive.setPositionPIDFCoefficients(7);
//        fl_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
////        fl_Drive.setPositionPIDFCoefficients(7);
//        br_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
////        br_Drive.setPositionPIDFCoefficients(7);
//        bl_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
////        bl_Drive.setPositionPIDFCoefficients(7);

        if (useBrake) {
            fl_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        l_roller.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        r_roller.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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
    public void driveTo(double x, double y, double z, Orientation rotation) {

    }

    public void drive(double straight, double side, double turn, double speedTrigger, double turnTrigger) {

        double speedBoost = speedTrigger * 0.5 + 0.5;
        double turnBoost = turnTrigger * 0.5 + 0.5;
        if (turnTrigger > 0.4 && speedTrigger > 0.4) {
            speedBoost = 1;
            turnBoost = 1;
        }
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
        boolean oldMode = fl_Drive.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION;
        boostState.update(speedTrigger > 0.7 || turnTrigger > 0.85);
        if (boostState.isChanged() || oldMode) {
            if (boostState.isPressed()) {
                fl_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                fr_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                bl_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                br_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                fl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                fr_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                bl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                br_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        int dir = 1;
        if (direction == Direction.REVERSE)
            dir = -1;
        int ticks = (int) (1400 * target_meter) * dir;

        int fl_tar_pos = fl_Drive.getCurrentPosition() + ticks;
        int fr_tar_pos = fr_Drive.getCurrentPosition() + ticks;
        int bl_tar_pos = bl_Drive.getCurrentPosition() + ticks;
        int br_tar_pos = br_Drive.getCurrentPosition() + ticks;


        fl_Drive.setTargetPosition(fl_tar_pos);
        fr_Drive.setTargetPosition(fr_tar_pos);
        bl_Drive.setTargetPosition(bl_tar_pos);
        br_Drive.setTargetPosition(br_tar_pos);

        fl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = drivePower;

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive() && runTime.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if (distFromStart < 300) {
                power = drivePower * distFromStart / 300 * 0.8 + 0.2;
            } else if (distToTarget < 500) {
                power = drivePower/3;
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
            opMode.sleep(5);
        }
    }

    public void straight_ignoreBumper(double target_meter, Direction direction, double speed, int timeout) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        int dir = 1;
        if (direction == Direction.REVERSE)
            dir = -1;
        int ticks = (int) (1400 * target_meter) * dir;

        int fl_tar_pos = fl_Drive.getCurrentPosition() + ticks;
        int fr_tar_pos = fr_Drive.getCurrentPosition() + ticks;
        int bl_tar_pos = bl_Drive.getCurrentPosition() + ticks;
        int br_tar_pos = br_Drive.getCurrentPosition() + ticks;


        fl_Drive.setTargetPosition(fl_tar_pos);
        fr_Drive.setTargetPosition(fr_tar_pos);
        bl_Drive.setTargetPosition(bl_tar_pos);
        br_Drive.setTargetPosition(br_tar_pos);

        fl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = drivePower;

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive() && runTime.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if (distFromStart < 300) {
                power = drivePower * distFromStart / 300 * 0.8 + 0.2;
            } else if (distToTarget < 500) {
                power = 0.2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);

            opMode.sleep(5);
        }
    }

    public void side(double target_meter, Direction direction, double speed, int timeout) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        int dir = 1;
        if (direction == Direction.LEFT)
            dir = -1;
        int ticks = (int) (1400 * target_meter) * dir;

        int fl_tar_pos = fl_Drive.getCurrentPosition() + ticks;
        int fr_tar_pos = fr_Drive.getCurrentPosition() - ticks;
        int bl_tar_pos = bl_Drive.getCurrentPosition() - ticks;
        int br_tar_pos = br_Drive.getCurrentPosition() + ticks;

        fl_Drive.setTargetPosition(fl_tar_pos);
        fr_Drive.setTargetPosition(fr_tar_pos);
        bl_Drive.setTargetPosition(bl_tar_pos);
        br_Drive.setTargetPosition(br_tar_pos);

        fl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power;

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) &&
                opMode.opModeIsActive() && runTime.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if (distFromStart < 500) {
                power = drivePower * distFromStart / 500 * 0.8 + 0.2;
            } else if (distToTarget < 500) {
                power = 0.2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);

            opMode.sleep(5);
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

        /* 1 round is 5600 ticks
         * to the right posotive
         * to the left negative*/
        int dir = 1;
        if (direction == Direction.LEFT)
            dir = -1;
        int ticks = (int) (5600 * rounds) * dir;

        int fl_tar_pos = fl_Drive.getCurrentPosition() + ticks;
        int fr_tar_pos = fr_Drive.getCurrentPosition() - ticks;
        int bl_tar_pos = bl_Drive.getCurrentPosition() + ticks;
        int br_tar_pos = br_Drive.getCurrentPosition() - ticks;

        fl_Drive.setTargetPosition(fl_tar_pos);
        fr_Drive.setTargetPosition(fr_tar_pos);
        bl_Drive.setTargetPosition(bl_tar_pos);
        br_Drive.setTargetPosition(br_tar_pos);

        fl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double drivePower = speed;
        double power = speed;

        while ((fl_Drive.isBusy() /*|| bl_Drive.isBusy() ||  fr_Drive.isBusy() || br_Drive.isBusy()*/) && opMode.opModeIsActive() && runtime.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if ((distFromStart < 200) || (distToTarget < 500)) {
                power = drivePower / 2;
            } else {
                power = drivePower;
            }

            fl_Drive.setPower(power);
            fr_Drive.setPower(power);
            bl_Drive.setPower(power);
            br_Drive.setPower(power);

            opMode.sleep(5);
        }
    }

    public void stop() {
        RobotLog.d("DriveClass:Stop()");
        fl_Drive.setPower(0);
        fr_Drive.setPower(0);
        bl_Drive.setPower(0);
        br_Drive.setPower(0);

        if (fl_Drive.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            fl_Drive.setTargetPosition(fl_Drive.getCurrentPosition());
            fr_Drive.setTargetPosition(fr_Drive.getCurrentPosition());
            bl_Drive.setTargetPosition(bl_Drive.getCurrentPosition());
            br_Drive.setTargetPosition(br_Drive.getCurrentPosition());
        }
    }

    public void end() {
        RobotLog.d("DriveClass:End()");
        fl_Drive.setPower(0);
        fr_Drive.setPower(0);
        bl_Drive.setPower(0);
        br_Drive.setPower(0);

        fl_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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

    public boolean getRollersState() {
        return rollerServoState.getState();
    }

    public void rollers(boolean open) {
        if (open) {

            r_roller_servo.setPosition(1);
            l_roller_servo.setPosition(0);
            rollerServoState.set(true);

        } else {
            r_roller_servo.setPosition(0);
            l_roller_servo.setPosition(1);
            rollerServoState.set(false);
        }
    }

    public double getRollersPower() {
        return r_roller.getPower();
    }

    public void rollersRunIn() {
        double rollerPower = 1;
        if (stoneBumper.getState() == false) {
            rollerPower = 0.1;
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

    public double getSensorDistance(Location location) {
        double result;
        if (location == Location.LEFT) {
            result = getSensorDistanceLeft();
            RobotLog.d("DriveClass: Left SensorDistance: %f", result);
        } else {
            result = getSensorDistanceRight() - 4;
            RobotLog.d("DriveClass: Right SensorDistance: %f", result);
        }
        return result;
    }

    public boolean isSkystone(Location location) {
        if (location == Location.LEFT) {
            return isSkyStone(sensorColorLeft);
        } else {
            return isSkyStone(sensorColorRight);
        }
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
        boolean skystone = hue > 110 && a > 500;
        // opMode.telemetry.addData("Sky:", "%b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", skystone, hue, r, g, b, a);

        RobotLog.d("Skystone found: %b - hue:%f, alpha:%f",skystone, hue, a);

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

    public boolean isLine(Alliance alliance) {
        if (alliance == Alliance.RED) {
            return isRed();
        } else {
            return isBlue();
        }
    }

    public boolean isRed() {
        ColorSensor sensor = sensorColorDown;
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double a = sensor.alpha();

        float hsvValues[] = {0F, 0F, 0F};
        final int SCALE_FACTOR = 255;

        Color.RGBToHSV( (int) (r * SCALE_FACTOR),
                        (int) (g * SCALE_FACTOR),
                        (int) (b * SCALE_FACTOR),
                        hsvValues);

        float hue = hsvValues[0];
        boolean red = Math.abs(hue) < 85;
        RobotLog.d("Red: %b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", red, hue, r, g, b, a);
        return red;
    }

    public boolean isBlue() {
        ColorSensor sensor = sensorColorDown;
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double a = sensor.alpha();

        float hsvValues[] = {0F, 0F, 0F};
        final int SCALE_FACTOR = 255;

        Color.RGBToHSV( (int) (r * SCALE_FACTOR),
                        (int) (g * SCALE_FACTOR),
                        (int) (b * SCALE_FACTOR),
                        hsvValues);

        float hue = hsvValues[0];
        boolean blue = hue > 160 && hue < 260;
        RobotLog.d("isBlue: %b - H: %03.02f, [R:%1.0f, G:%1.0f, B:%1.0f, A:%1.0f]", blue, hue, r, g, b, a);
        return blue;
    }

    void reset() {
        fl_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void back() {
        fl_Drive.setTargetPosition(0);
        fr_Drive.setTargetPosition(0);
        bl_Drive.setTargetPosition(0);
        br_Drive.setTargetPosition(0);
    }


    //===============================================================================================================================================================================================================================================
    //Autonomous Functions
    //===============================================================================================================================================================================================================================================

    public enum Alliance { BLUE, RED }

    public void AUTO_foundation(Alliance team, boolean wall) {
        int mul = 0;
        if (team == Alliance.BLUE) {
            mul = 1;
        }
        if (team == Alliance.RED) {
            mul = -1;
        }

        side(0.6 * mul, Direction.RIGHT, 0.7, 3);
        straight(1.3, Direction.REVERSE, 0.7, 4);
        straight(0.5, Direction.REVERSE, 0.2, 5);

        // sleep(200);
        // Step 2: should be in front of the foundation, hooks down
        drive(-0.2, 0 * mul, 0, 0, 0);
        hooksDown();
        opMode.sleep(500);
        stop();
        // Step 3: drag the foundation to the wall
        straight(1.8, Direction.FORWARD, 0.7, 3);

        // Step 4: set the foundation free
        hooksUp();
        opMode.sleep(100);
        /////////////////////////////////////////////////////////////////////////////////////
        // Step 5: a)go to the side of the foundation b)push it to the wall c)go in front of the foundation d) move it to the wall
        //  |
        // a|    <---|
        //  |b ^|---->
        //  V-->|  c
        //      |
        //======V==============||||||||||||========================
        //
        /////////////////////////////////////////////////////////////////////////////////////
        side(1.8 * mul, Direction.LEFT,  0.7, 3); // slide out
        straight(0.8, Direction.REVERSE, 0.8, 3);
        side(0.8 * mul, Direction.RIGHT, 0.9, 3); // Push foundation to side wall
        straight(1.0, Direction.REVERSE, 0.9, 3);
        side(1.2 * mul, Direction.RIGHT, 0.9, 3);
        straight(1.2, Direction.FORWARD, 0.9, 3); // Push foundation to wall
        // Step 6: drive to the side - park under the bridge

        if (wall == false) { // park cloth to Bridge
            straight(0.3, Direction.REVERSE, 0.5, 2);
            side(2.2 * mul, Direction.LEFT, 0.9, 3);
        }

        if (wall == true) { // park cloth to Wall
            side(1.8 * mul, Direction.LEFT, 0.9, 3);
            straight(1.2, DriveClass.Direction.FORWARD, 0.5, 2);
            side(0.6 * mul, Direction.LEFT, 0.9, 3);
        }

        driveToLine(team, false);
    }

    public void searchSkystone(Location location,int mul) {
        ElapsedTime timer = new ElapsedTime();
        // drive LEFT : search for SkyStone
        drive(0, -0.2 * mul, 0, 0, 0);

        timer.reset();
        double speed = 0.2;
        while (!isSkystone(location) && timer.seconds() < 4  && opMode.opModeIsActive()) {
            drive(0, -speed * mul, 0, 0, 0);
            opMode.sleep(1);
            speed += 0.002;
            if (speed<0.8){
                speed=0.8;
            }
        }
    }

    public void driveToLine(Alliance team, boolean skystone) {
        // drive LEFT : back to line.
        int mul = (team == Alliance.RED) ? 1 : -1;
        if (skystone) mul *= -1;

        ElapsedTime timer = new ElapsedTime();
        drive(0, 0.3 * mul, 0, 0, 0);
        timer.reset();
        while (!isLine(team) && timer.seconds() < 3  && opMode.opModeIsActive()) {
            //telemetry.update();
            opMode.sleep(1);
        }
    }

    public void AUTO_skystone(Alliance team, ArmClass arm, boolean foundation) {
        ElapsedTime timer = new ElapsedTime();
        int mul;
        Location location;

        if (team == Alliance.BLUE) {
            mul = -1;
            location = Location.RIGHT;
        } else { // Alliance.BLUE
            mul = 1;
            location = Location.LEFT;
        }

        try {
            RobotLog.d("First Skystone");
            //robot.side(0.6, DriveClass.Direction.RIGHT, 1, 3);
            arm.pleaseDo(ArmClass.Mode.SKY1);
            straight(0.9, DriveClass.Direction.FORWARD, 1, 4);

            // drive straight close to stones
            drive(0.3, 0, 0, 0, 0);
            timer.reset();
            while (getSensorDistance(location) > 3 && timer.seconds() < 3 && opMode.opModeIsActive()) {
                opMode.sleep(1);
            }
            stop();

            searchSkystone(location,mul);

            // drive LEFT one block
            side(0.24 * mul, DriveClass.Direction.LEFT, 0.5, 2);

            // catch STONE
            arm.setArmDriveMode(false);
            arm.gootoo(50, 400,1);
            arm.clamp(false); // close clamps
            opMode.sleep(800); // wait for clamps to close
            arm.pleaseDo(ArmClass.Mode.SKY2); // move arm back

            // drive backwards
            straight(0.2, DriveClass.Direction.REVERSE, 1, 1);

            // slide RIGHT to put stone
            side(2.8 * mul, DriveClass.Direction.RIGHT, 0.95, 8);
            // robot.stop();

            // Dropdown stone.
            arm.setArmDriveMode(false);
            arm.gootoo(ArmClass.STAY,700);
            arm.clamp(true);
            opMode.sleep(200);
            arm.gootoo(ArmClass.STAY,0);
            arm.clamp(false);

            // ============= Second Skystone ==============================================================================================================================
            RobotLog.d("Second Skystone");
            side(3.3 * mul, DriveClass.Direction.LEFT, 0.95, 8);

            arm.clamp(true);
            arm.pleaseDo(ArmClass.Mode.SKY3); // move arm forward ready to catch
            //straight(0.2, DriveClass.Direction.FORWARD, 0.5, 1);
            stop();

            // drive straight close to stones

            drive(0.3, 0, 0, 0, 0);
            timer.reset();
            while (getSensorDistance(location) > 3 && timer.seconds() < 3 && opMode.opModeIsActive()) {
                opMode.sleep(1);
            }
            stop();

            // drive LEFT : search for SkyStone
            searchSkystone(location,mul);

            // drive LEFT one block
            side(0.24 * mul, DriveClass.Direction.LEFT, 0.4, 2);
            //  robot.stop();

            // catch STONE
            arm.setArmDriveMode(false);
            arm.gootoo(50, 400,1.5);
            arm.clamp(false);
            opMode.sleep(1000);
            arm.pleaseDo(ArmClass.Mode.SKY2);

            // drive backwards
            straight(0.25, DriveClass.Direction.REVERSE, 1, 1);

            if(foundation == true){

                side(5 * mul, DriveClass.Direction.RIGHT, 0.95, 8);
                straight(0.3, Direction.FORWARD,1,3);
                arm.setArmDriveMode(false);
                arm.gootoo(700,900);
                arm.clamp(true);
                straight(0.5, Direction.REVERSE,1,3);
                arm.pleaseDo(ArmClass.Mode.HOME);
                arm.clamp(false);
                rotate(0.5,Direction.LEFT,1,3);
                straight(0.5, Direction.REVERSE,1,3);
                hooksDown();
                straight(0.3, Direction.REVERSE,1,3);
                straight(1.2, Direction.FORWARD,1,5);
                rotate(0.25,Direction.LEFT,1,3);
                hooksUp();
            }
            else {
                // slide RIGHT to put stone
                side(3.8 * mul, DriveClass.Direction.RIGHT, 0.9, 8);
                //    robot.stop();

                arm.clamp(true);
                arm.setArmDriveMode(false);
                arm.gootoo(ArmClass.STAY, 0, 1);
                arm.clamp(false);

                arm.linearDo(ArmClass.Mode.HOME);
                side(-0.7 * mul, DriveClass.Direction.RIGHT, 0.7, 8);


                driveToLine(team, true);
            }
            stop();

        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
            stop();
        }
    }
}


