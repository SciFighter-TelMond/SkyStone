package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    // The Gyro IMU sensor object
    BNO055IMU imu;

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

        fl_Drive.setTargetPositionTolerance(40);
        fr_Drive.setTargetPositionTolerance(40);
        bl_Drive.setTargetPositionTolerance(40);
        br_Drive.setTargetPositionTolerance(40);

        PIDFCoefficients pidf = fr_Drive.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Drive PID");
        RobotLog.d(pidf.toString());
        // p=9.999847 i=2.999954 d=0.000000 f=0.000000

        pidf.p=5;   // 10;
        pidf.i=6;    // 3;
        pidf.d=0;  // 0

        fr_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
//        fr_Drive.setPositionPIDFCoefficients(7);
        fl_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
//        fl_Drive.setPositionPIDFCoefficients(7);
        br_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
//        br_Drive.setPositionPIDFCoefficients(7);
        bl_Drive.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
//        bl_Drive.setPositionPIDFCoefficients(7);

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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void init_GyroIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // parameters.loggingEnabled      = false;
        // parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        // Start the logging of measured acceleration
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

//        opMode.telemetry.addLine()
//                .addData("IMU:", new Func<String>()     { @Override public String value() { return imu.getSystemStatus().toShortString(); }})
//                .addData("Cal:", new Func<String>()     { @Override public String value() { return imu.getCalibrationStatus().toString(); }})
//                .addData("Heading:", new Func<String>() { @Override public String value() { return Float.toString(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle); }});
//

        while (imu.isGyroCalibrated() && !opMode.isStopRequested()) {
            opMode.sleep(100);
            opMode.telemetry.update();
        }
        if (imu.isGyroCalibrated())
            opMode.telemetry.addData("Status", "Gyro IMU Ready");
        else
            opMode.telemetry.addData("Status", "Gyro IMU FAILED !!!!!!!!!!!!!!");

        opMode.telemetry.update();

        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
        RobotLog.d("IMU calib: %s", imu.getCalibrationStatus().toString());
    }

    public boolean getStoneBumperState() {
        return stoneBumper.getState();
    }

    public double readGyroHeading(double targetAngle) {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double angle = -angles.firstAngle;
        double deltaAngle = targetAngle - angle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        double globalAngle = targetAngle - deltaAngle;

        return globalAngle;
    }

    // ==================================================================================================

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

    public void straight(double target_meter, Direction direction, double speed, double timeout, double heading, boolean stopOnBumpers) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

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

        double power;
        int accDist = 500;

        while ((fl_Drive.isBusy() && fr_Drive.isBusy()) && opMode.opModeIsActive() && timer.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if (distFromStart < accDist) {
                power = (speed-0.2) * distFromStart / accDist * 0.8 + 0.2;
            } else if (distToTarget < accDist) {
                power = (speed-0.2) * distToTarget / accDist * 0.8 + 0.2;
            } else {
                power = speed;
            }

            // Gyro Heading Correction
            double headingGyro  = readGyroHeading(heading);
            double headingError = heading - headingGyro;
            double headingCorrection = headingError * 0.06 * power * dir * Math.signum(target_meter);;
            if (distToTarget < 300)
                headingCorrection = 0;

            fl_Drive.setPower(power + headingCorrection);
            fr_Drive.setPower(power - headingCorrection);
            bl_Drive.setPower(power + headingCorrection);
            br_Drive.setPower(power - headingCorrection);

//            fl_Drive.setPower(power + headingCorrection);
//            fr_Drive.setPower(power + headingCorrection);
//            bl_Drive.setPower(power - headingCorrection);
//            br_Drive.setPower(power - headingCorrection);

            RobotLog.d("%f2.4 ] straight: power: %f, heading: %f, corr: %f", timer.seconds(), power, headingGyro, headingCorrection );

            if (stopOnBumpers && direction == Direction.REVERSE) {
                if (leftBumper.getState() == false) {
                    fl_Drive.setTargetPosition(fl_Drive.getCurrentPosition());
                    bl_Drive.setTargetPosition(bl_Drive.getCurrentPosition());
                }

                if (rightBumper.getState() == false) {
                    fr_Drive.setTargetPosition(fr_Drive.getCurrentPosition());
                    br_Drive.setTargetPosition(br_Drive.getCurrentPosition());
                }
            }
            opMode.sleep(1);
        }
    }

    public void strafe(double target_meter, Direction direction, double speed, int timeout, double heading) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

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

        double power;
        int accDist = 500;

        while ((fl_Drive.isBusy() && fr_Drive.isBusy()) && opMode.opModeIsActive() && timer.seconds() < timeout) {

            int distToTarget = Math.abs(fr_tar_pos - fr_Drive.getCurrentPosition());

            int distFromStart = Math.abs(ticks - distToTarget);

            if (distFromStart < accDist) {
                power = (speed-0.2) * distFromStart / accDist * 0.8 + 0.2;
            } else if (distToTarget < accDist) {
                power = (speed-0.2) * distToTarget / accDist * 0.8 + 0.2;
            } else {
                power = speed;
            }

            // Gyro Heading Correction
            double headingGyro = readGyroHeading(heading);
            double headingError = heading - headingGyro;
            double headingCorrection = dir * headingError * 0.06 * power * Math.signum(target_meter);
            if (distToTarget < 300)
                headingCorrection = 0;
            RobotLog.d("%f2.4 ] strafe: power: %f, heading: %f, corr: %f", timer.seconds(), power, headingGyro, headingCorrection );

            fl_Drive.setPower(power + headingCorrection);
            fr_Drive.setPower(power + headingCorrection);
            bl_Drive.setPower(power - headingCorrection);
            br_Drive.setPower(power - headingCorrection);

            opMode.sleep(1);
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

    public void rotateTo(double targetAngle, double speed, double timeout, double tollerance) {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double currAngle  = readGyroHeading(targetAngle);
        double deltaAngle = targetAngle - currAngle;

        /* 1 round is 5600 ticks
         * to the right positive
         * to the left negative*/
        double rounds = deltaAngle / 360 ;
        rotate(rounds, Direction.RIGHT, speed, timeout );

        fl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double power = speed;

        currAngle  = readGyroHeading(targetAngle);
        deltaAngle = targetAngle - currAngle;
        RobotLog.d("%f2.4 ] rotateTo: power: %f, target: %f, gyro: %f, corr: %f", timer.seconds(), power, targetAngle, currAngle, deltaAngle );
        while ( (Math.abs(deltaAngle) > tollerance) && opMode.opModeIsActive() && timer.seconds() < timeout) {

            double error = deltaAngle;
            double gain = 0.01;
            power =  Math.min(speed, Math.max(0.0, Math.abs(error) * gain)) * Math.signum(error);
            fl_Drive.setPower(+power);
            fr_Drive.setPower(-power);
            bl_Drive.setPower(+power);
            br_Drive.setPower(-power);

            opMode.sleep(1);

            currAngle  = readGyroHeading(targetAngle);
            deltaAngle = targetAngle - currAngle;
            RobotLog.d("%f2.4 ] rotateTo: power: %f, target: %f, curr: %f, delta: %f,", timer.seconds(), power, targetAngle, currAngle, deltaAngle );
        }
        stop();
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
        return sensorDistanceRight.getDistance(DistanceUnit.CM) - 4;
    }

    public double getSensorDistanceLeft() {
        return sensorDistanceLeft.getDistance(DistanceUnit.CM);
    }

    public double getSensorDistance(Location location) {
        double dist;
        if (location == Location.LEFT) {
            dist = getSensorDistanceLeft();
            RobotLog.d("DriveClass: Left SensorDistance: %f", dist);
        } else {
            dist = getSensorDistanceRight(); // TODO: the sensor is missed calibrate
            RobotLog.d("DriveClass: Right SensorDistance: %f", dist);
//            dist -= 4;
        }
        return dist;
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

        Color.RGBToHSV( (int) (r * SCALE_FACTOR),
                        (int) (g * SCALE_FACTOR),
                        (int) (b * SCALE_FACTOR),
                        hsvValues);

        float hue = hsvValues[0];
        float s = hsvValues[1];
        float v = hsvValues[2];
        boolean skystone = hue > 110 && a > 500 && s < 0.65; //&& v < 600;

        RobotLog.d("Skystone found: %b - hue:%f, s:%f, v:%f, alpha:%f",skystone, hue, s, v, a);

        return skystone;
    }

    public void setCapstone(boolean open) {
        if (open == true) {
            capstone.setPosition(1);
        }

        if (open == false) {
            capstone.setPosition(0);
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
    public enum FoundationType { WALL, BRIDGE, SHORT, PARK_ONLY }

    public void AUTO_foundation(Alliance team, FoundationType foundationType) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int mul = 0;
        if (team == Alliance.BLUE) {
            mul = 1;
        }
        if (team == Alliance.RED) {
            mul = -1;
        }

        if (foundationType == FoundationType.PARK_ONLY) {
            while (timer.seconds() < 20) {
                opMode.sleep(100);
            }
            strafe(1.5 * mul, Direction.LEFT, 0.7, 3, 0);
        }
        else {
            strafe(0.7 * mul, Direction.RIGHT, 0.7, 3, 0);
            straight(1.3, Direction.REVERSE, 0.7, 4, 0, true);
            catchFoundation(0);
            opMode.sleep(300);
            // Step 3: drag the foundation to the wall
            straight(1.8, Direction.FORWARD, 0.7, 3, 0, false);

            // Step 4: set the foundation free
            hooksUp();
            opMode.sleep(100);
            /////////////////////////////////////////////////////////////////////////////////////
            // Step 5: a)go to the strafe of the foundation b)push it to the wall c)go in front of the foundation d) move it to the wall
            //  |
            // a|    <---|
            //  |b ^|---->
            //  V-->|  c
            //      |
            //======V==============||||||||||||========================
            //
            /////////////////////////////////////////////////////////////////////////////////////
            strafe(1.8 * mul, Direction.LEFT, 0.7, 3, 0); // slide out

            if (foundationType != FoundationType.SHORT) {
                straight(0.8, Direction.REVERSE, 0.8, 3, 0, false);
                strafe(0.8 * mul, Direction.RIGHT, 0.9, 3, 0); // Push foundation to strafe wall
                straight(1.1, Direction.REVERSE, 0.9, 3, 0, false);
                strafe(1.2 * mul, Direction.RIGHT, 0.9, 3, 0);
                straight(1.1, Direction.FORWARD, 0.9, 3, 0, false); // Push foundation to wall
                // Step 6: drive to the strafe - park under the bridge

                if (foundationType == FoundationType.BRIDGE) { // park cloth to Bridge
                    straight(0.35, Direction.REVERSE, 0.5, 2, 0, false);
                    strafe(2.2 * mul, Direction.LEFT, 0.9, 3, 0);
                }

                if (foundationType == FoundationType.WALL) { // park cloth to Wall
                    strafe(1.8 * mul, Direction.LEFT, 0.9, 3, 0);
                    straight(1.2, DriveClass.Direction.FORWARD, 0.5, 2, 0, false);
                    strafe(0.6 * mul, Direction.LEFT, 0.9, 3, 0);
                }
            }
        }
        driveToLine(team, false);
    }


    public void straightenUp() {
        // drive straight close to stones
        ElapsedTime timer = new ElapsedTime();
        double speed = 0.1;
        double diff = 1;
        timer.reset();
        while ( (Math.abs(diff) > 0.1) && (timer.seconds() < 3) && opMode.opModeIsActive()) {
            double left  = getSensorDistance(Location.LEFT);
            double right = getSensorDistance(Location.RIGHT);
            diff = left - right;
            drive(0, 0, diff * speed, 0, 0);
            RobotLog.d("Straiting up diff:%f, left:%f, right:%f", diff, left, right);
            opMode.sleep(1);
        }
        stop();
    }

    public double approachStones(Location location) {
        // drive straight close to stones
        ElapsedTime timer = new ElapsedTime();
        double speed = 0.5;
        drive(speed, 0, 0, 0, 0);
        timer.reset();
        double dist = getSensorDistance(location);
        while ( (dist > 3) && (timer.seconds()) < 3 && opMode.opModeIsActive()) {
            if (dist < 4) {
                speed = (dist - 2)/2 * 0.5;
                drive(speed, 0, 0, 0, 0);
            }
            opMode.sleep(5);
            dist = getSensorDistance(location);
        }
        stop();
        return dist;
    }

    private double speedup( double time, double min, double max, double accTime ) {
        double speed = time / accTime * max;
        speed = Math.max(min, speed);
        speed = Math.min(max, speed);
        return speed;
    }

    private double searchSkystone(Location location,int mul) {
        // drive side way : search for SkyStone and return the time to find it.
        ElapsedTime total_timer = new ElapsedTime();
        total_timer.reset();
        ElapsedTime timer = new ElapsedTime();
        double startTick = fl_Drive.getCurrentPosition();
        double maxSpeed = 0.8;
        double accTime = 0.4;
        double speed = 0.2;
        drive(0, -0.2 * mul, 0, 0, 0);
        timer.reset();
        // search for SkyStone
        while (!isSkystone(location) && timer.seconds() < 4  && opMode.opModeIsActive()) {
            speed = speedup(total_timer.seconds(), 0.2, maxSpeed, accTime);
            drive(0, -speed * mul, 0, 0, 0);
            opMode.sleep(2);
        }
        RobotLog.d("Skystone 1 at %f sec",total_timer.seconds());

//        // continue driving until cant see the skystone
//        timer.reset();
//        while (isSkystone(location) && timer.seconds() < 2  && opMode.opModeIsActive()) {
//            speed = speedup(total_timer.seconds(), 0.2, maxSpeed, accTime);
//            drive(0, -speed * mul, 0, 0, 0);
//            RobotLog.d("Skystone speed %f",speed);
//            opMode.sleep(2);
//        }
//        RobotLog.d("Skystone 2 at %f sec",total_timer.seconds());

        // continue driving a little more to middle of the stone
        int startPos = fl_Drive.getCurrentPosition();
        int ticks = (int) (1400 * 0.24);
        // int target = startPos - ticks * mul;
        timer.reset();
        while (Math.abs(fl_Drive.getCurrentPosition() - startPos) < Math.abs(ticks) && timer.seconds() < 0.6  && opMode.opModeIsActive()) {
            speed = speedup(total_timer.seconds(), 0.2, maxSpeed, accTime);
            drive(0, -speed * mul, 0, 0, 0);
            opMode.sleep(2);
        }
        stop();
        double sec = total_timer.seconds();
        double dist = (double)Math.abs(fl_Drive.getCurrentPosition() - startPos) / 1400;
        RobotLog.d("Skystone found after %f sec, dist: %f",sec, dist);
        return dist;
    }

    private void driveToLine(Alliance team, boolean skystone) {
        // drive LEFT : back to line.
        int mul = (team == Alliance.RED) ? 1 : -1;
        if (skystone) mul *= -1;

        ElapsedTime timer = new ElapsedTime();
        drive(0, 0.4 * mul, 0, 0, 0);
        timer.reset();
        while (!isLine(team) && timer.seconds() < 3  && opMode.opModeIsActive()) {
            //telemetry.update();
            opMode.sleep(1);
        }
    }

    public void catchFoundation(double heading) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        drive(-0.8, 0, 0, 0, 0);
        while( (leftBumper.getState() == true) && (rightBumper.getState() == true) && (timer.seconds() < 3) && opMode.opModeIsActive() ) {
            opMode.sleep(2);
        }
        hooksDown();                                                         // catch the foundation
        timer.reset();
        drive(-0.1, 0, 0, 0, 0);
        while( !((leftBumper.getState() == false) && (rightBumper.getState() == false)) && (timer.seconds() < 1) && opMode.opModeIsActive() ) {
            opMode.sleep(2);
        }
        stop();
        straight(0.1, Direction.REVERSE, 0.2, 0.5, 180, false ); // slow drive to hook the foundation.
        opMode.sleep(200);
    }

    public void AUTO_skystone(Alliance team, ArmClass arm, boolean foundation) {
        ElapsedTime timer = new ElapsedTime();
        int mul;
        Location location;

        if (team == Alliance.BLUE) {
            mul = -1;
            location = Location.RIGHT;
        } else { // Alliance.RED
            mul = 1;
            location = Location.LEFT;
        }

        try {
            RobotLog.d("First Skystone");
            //robot.strafe(0.6, DriveClass.Direction.RIGHT, 1, 3, 0);
            arm.pleaseDo(ArmClass.Mode.SKY1_STRETCH);
            straight(0.9, DriveClass.Direction.FORWARD, 1, 4, 0,false);

            // find skystone
            approachStones(location);     // drive straight close to stones
            double skyDist = searchSkystone(location,mul); // drive LEFT : search for SkyStone
            RobotLog.d("skyDist: %f", skyDist);

            // pickup STONE
            arm.openClamps(false); // close clamps
            arm.setArmDriveMode(false);
            arm.gootoo(60, 400,0.4);
            opMode.sleep(300); // wait for clamps to close
            arm.pleaseDo(ArmClass.Mode.SKY2_FOLD); // move arm back

            // drive backwards
            straight(0.3, DriveClass.Direction.REVERSE, 1, 1, 0,false);

            // slide RIGHT to put stone
            strafe((2.3 + skyDist) * mul, DriveClass.Direction.RIGHT, 1, 8, 0);
            // robot.stop();

            // Dropdown stone.
            arm.pleaseDo(ArmClass.Mode.SKY4_DROP);
            opMode.sleep(800);

            // ============= Second Skystone ==============================================================================================================================
            RobotLog.d("Second Skystone");
            strafe((2.8 + skyDist)* mul, DriveClass.Direction.LEFT, 1, 8, 0); // Drive back to stones.

            arm.openClamps(true);
            arm.pleaseDo(ArmClass.Mode.SKY3_READY); // move arm forward ready to catch

            // find skystone
            approachStones(location);     // drive straight close to stones
            searchSkystone(location,mul); // drive LEFT : search for SkyStone

            // pickup STONE
            arm.openClamps(false);
            arm.setArmDriveMode(false);
            arm.gootoo(60, 400,0.4);
            opMode.sleep(300); // wait for clamps to close
            arm.pleaseDo(ArmClass.Mode.SKY2_FOLD);

            // drive backwards
            straight(0.3, DriveClass.Direction.REVERSE, 1, 1, 0,false);

            if(foundation == true) { // SOLO

                strafe((4.8 + skyDist) * mul, Direction.RIGHT, 1, 8, 0); // slide toward foundation line

                rotateTo(180 * mul,1,3, 3);      // rotate 180
                // straight(0.4, Direction.REVERSE,1,3, 180, true);    // drive to foundation
                catchFoundation(180);
                arm.pleaseDo(ArmClass.Mode.SKY5_DROP_BACK);                          // drop the stone backwards and HOME the arm while moving the foundation.

                // Rotate the foundation to the building zone.
                rotate(0.05 * mul, Direction.RIGHT,1,0.5);       // rotate 15c - just a little to gain angle.

                double angle = 180 + 20 * mul ;
                straight(0.6, Direction.FORWARD,1,3, angle, false);  // move forward half way - move foundation from the wal.

                double bridgeDirection =  - 90 * mul ; // bridge direction
                rotateTo(bridgeDirection,1,3, 5);      // rotate foundation to building zone.
                hooksUp();                                                      // release foundation.
                opMode.sleep(150);                                  // wait for hooks to open

                // Parking - drive FORWARD towards bridge line to park - search the line and park.
                straight(0.7, Direction.FORWARD,1,3, bridgeDirection, false);       // move forward toward bridge.
                timer.reset();
                drive(0.5, 0, 0, 0, 0); // drive slowly until it sees the line.
                while (!isLine(team) && timer.seconds() < 3  && opMode.opModeIsActive()) {
                    opMode.sleep(1);
                }
                stop();
                // waist time until the end to let the arm HOME finish.
                while (opMode.opModeIsActive()) {
                    opMode.sleep(10);
                }
            }
            else { // BRIDGE
                // slide RIGHT to put stone after bridge
                strafe(3.8 * mul, DriveClass.Direction.RIGHT, 0.9, 8, 0); // TODO: stoneDist

                arm.openClamps(true);
                arm.setArmDriveMode(false);
                arm.gootoo(ArmClass.STAY, 0, 1);
                arm.openClamps(false);

                arm.linearDo(ArmClass.Mode.HOME);
                strafe(-0.7 * mul, DriveClass.Direction.RIGHT, 0.7, 8, 0);
                opMode.sleep(50);

                driveToLine(team, true);
            }
            stop();

        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
            stop();
        }
    }
}


