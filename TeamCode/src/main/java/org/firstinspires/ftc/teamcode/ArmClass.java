package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmClass extends Thread {

    volatile private boolean stopFlag = true;

    volatile private DcMotor arm0 = null;
    volatile private DcMotor arm1 = null;
    volatile private DigitalChannel zeroArm0 = null;
    volatile private DigitalChannel zeroArm1a = null;
    volatile private DigitalChannel zeroArm1b = null;

    volatile private Servo clamps = null;
    volatile private Servo clampsRotate = null;

    volatile private double power = 1;
    volatile private double speed_boost = 0.5;

    public enum Mode {IDLE, MANUAL, HOME, PICK, STRAIGHT, BUILD, DROP}

    volatile private Mode mode = Mode.IDLE;

    volatile private int posArm0 = 0;
    volatile private int posArm1 = 0;

    volatile private OpMode opMode = null;
    volatile private DriveClass driveClass = null;

    static final int STAY = 999999;

    public ArmClass(OpMode opMode, DriveClass drive) {
        this.setName("ArmClass");
        RobotLog.d("%s", this.getName());

        this.opMode = opMode;
        this.driveClass = drive;
    }

    public void init(HardwareMap hardwareMap) {
        arm0 = hardwareMap.get(DcMotor.class, "arm0");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        zeroArm0 = hardwareMap.get(DigitalChannel.class, "zero_arm0");
        zeroArm1a = hardwareMap.get(DigitalChannel.class, "zero_arm1"); // TODO change names
        zeroArm1b = hardwareMap.get(DigitalChannel.class, "end_arm1"); // TODO change names

        clamps = hardwareMap.get(Servo.class, "clamps");
        clampsRotate = hardwareMap.get(Servo.class, "clamps_rotate");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1a.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1b.setMode(DigitalChannel.Mode.INPUT);
        arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void begin() {
        reset();
        mode = Mode.MANUAL;
        stopFlag = false;
    }

    public void reset() {
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setModeArm0(true);
        setModeArm1(true);
    }

    private void setModeArm0(boolean drive) {
        if (drive) {
            arm0.setPower(0);
            arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            arm0.setTargetPosition(arm0.getCurrentPosition());
            arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm0.setPower(power);
        }
    }

    private void setModeArm1(boolean drive) {
        if (drive) {
            arm1.setPower(0);
            arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            arm1.setTargetPosition(arm1.getCurrentPosition());
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm1.setPower(power);
        }
    }

    public void setBoost(double boost) {
        speed_boost = 0.3 + boost * 0.7;
    }

    public void moveArm0(double speed) {
        if (mode != Mode.MANUAL)
            return;

        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        }

        arm0.setPower(speed * speed_boost);
    }

    public void moveArm1(double speed) {
        if (mode != Mode.MANUAL)
            return;
//        if (zeroArm1a.getState() == true && zeroArm1b.getState() == true) {
//           speed = Math.max(0, speed);
//        }

        arm1.setPower(speed * speed_boost);
    }

    public void checkups() {
        if (zeroArm0.getState() == false) {
            arm0.setTargetPosition(Math.max(arm0.getCurrentPosition(), arm0.getTargetPosition()));
        }
        if (zeroArm1a.getState() == true && zeroArm1b.getState() == true) {
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }

        if (Math.abs(arm0.getCurrentPosition() - posArm0) <= 10){
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }
        if (Math.abs(arm1.getCurrentPosition() - posArm1) <= 10){
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }

        posArm0 = arm0.getCurrentPosition();
        posArm1 = arm1.getCurrentPosition();


        if (opMode != null) {
            opMode.telemetry.addData("Arms Switch", "Arm0:(%b), Arm1 a:(%b), b:(%b)", zeroArm0.getState(), zeroArm1a.getState(), zeroArm1b.getState());
            opMode.telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            opMode.telemetry.update();
        }
    }

    public void gootoo(int pos0, int pos1) throws InterruptedException {
        if (pos0 == STAY) pos0 = arm0.getCurrentPosition();
        if (pos1 == STAY) pos1 = arm1.getCurrentPosition();

        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);
        while (arm0.isBusy()) {
            sleep(50);
            checkups();
        }
        while (arm1.isBusy()) {
            sleep(50);
            checkups();
        }
    }

    public void pleaseDo(Mode tmode) {
        interrupt();
        mode = tmode;
        if (tmode != Mode.MANUAL || tmode != Mode.IDLE) {
            start();
        }
    }

    public void linearDo(Mode tmode) {
        mode = tmode;
        if (tmode != Mode.MANUAL || tmode != Mode.IDLE) {
            run();
        }
    }

    @Override
    public void run() {
        try {
            power = 0.5; // TODO: remove
            setModeArm0(false);
            setModeArm1(false);
            switch (mode) {
                case HOME:
//                    if (arm0.getCurrentPosition() < 3000) {
//                        gootoo(3100, arm1.getCurrentPosition()); // 1
//                    }
                    gootoo(2400, 2150);
                    gootoo(0, -400);
                    break;

                case PICK:
                    // peak up a cube and get back to drive position.
                    gootoo(1000, STAY);
                    gootoo(2300, 2300);
                    gootoo(2282, 3115);
                    rotateClamp(false);
                    clamp(true);
                    if (driveClass != null)
                        driveClass.rollers(true);
//                    gootoo(2282, 2609);
                    gootoo(2210, 3540);
                    gootoo(1300, 3090);
                    clamp(false);
                    sleep(1000);
                    gootoo(2210, 3540);
                    break;

                case STRAIGHT:
                    gootoo(1000, STAY);
                    gootoo(2400, 2150);
                    break;

                default:
                    break;
            }
        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        } catch (Exception e) {
            RobotLog.logStackTrace(e);
        }
        setModeArm0(true);
        setModeArm1(true);
        mode = Mode.MANUAL;
    }

    public void end() {
        stopFlag = true;
        interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
        arm0.setTargetPosition(arm0.getCurrentPosition());
        arm1.setTargetPosition(arm1.getCurrentPosition());
        mode = Mode.IDLE;
    }

    public void resumePower() {
        arm0.setTargetPosition(arm0.getCurrentPosition());
        arm1.setTargetPosition(arm1.getCurrentPosition());
        arm0.setPower(power);
        arm1.setPower(power);
        mode = Mode.MANUAL;
        stopFlag = false;
    }

    public int getArm0Pos() {
        return arm0.getCurrentPosition();
    }

    public int getArm1Pos() {
        return arm1.getCurrentPosition();
    }

    public boolean getArm0Zero() {
        return zeroArm0.getState();
    }

    public boolean getArm1ZeroA() {
        return zeroArm1a.getState();
    }

    public boolean getArm1ZeroB() {
        return zeroArm1b.getState();
    }

    public void clamp(boolean open) {
        if (open) {
            clamps.setPosition(1);
        } else {
            clamps.setPosition(0);
        }
    }

    public void rotateClamp(boolean rot) {
        if (rot) {
            clampsRotate.setPosition(0);
        } else {
            clampsRotate.setPosition(1);
        }
    }
}