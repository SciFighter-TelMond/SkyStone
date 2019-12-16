package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmClass extends Thread {

    private boolean stopFlag = true;

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel zeroArm0 = null;
    private DigitalChannel zeroArm1a = null;
    private DigitalChannel zeroArm1b = null;

    private Servo clamps = null;
    private Servo clampsRotate = null;

    private int manualPos0 = 0;
    private int manualPos1 = 0;

    private double power = 1;
    private double motor_speed_k = 300;
    private double speed_boost = 0.5;

    public enum Mode {IDLE, MANUAL, HOME, PICK, STRAIGHT, BUILD, DROP}

    private Mode mode = Mode.IDLE;

    private boolean arm0Move = false;
    private boolean arm1Move = false;

    private OpMode opMode = null;
    private DriveClass driveClass = null;

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
            arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm0.setPower(0);
        } else {
            arm0.setTargetPosition(arm0.getCurrentPosition());
            arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm0.setPower(power);
        }
    }

    private void setModeArm1(boolean drive) {
        if (drive) {
            arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm1.setPower(0);
        } else {
            arm1.setTargetPosition(arm1.getCurrentPosition());
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm1.setPower(power);
        }
    }

    public void setBoost(double boost) {
        speed_boost = 0.5 + boost * 0.5;
    }

    public void moveArm0(double speed) {
        if (mode != Mode.MANUAL)
            return;

        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        }

//        if (!arm0Move && speed != 0) setModeArm0(true);
//        if (arm0Move && speed == 0) setModeArm0(false);

//        if (arm0Move || speed != 0) {
            arm0.setPower(speed * speed_boost);
//            arm0Move = speed != 0;
//        }
    }

    public void moveArm1(double speed) {
        if (mode != Mode.MANUAL)
            return;
//        if (zeroArm1a.getState() == true && zeroArm1b.getState() == true) {
//           speed = Math.max(0, speed);
//        }

//        if (!arm1Move && speed != 0) setModeArm1(true);
//        if (arm1Move && speed == 0) setModeArm1(false);

//        if (arm1Move || speed != 0) {
            arm1.setPower(speed * speed_boost);
//            arm1Move = speed != 0;
//        }
    }

    public void checkups() {
        if (zeroArm0.getState() == false) {
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }
        if (zeroArm1a.getState() == true && zeroArm1b.getState() == true) {
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }

        if (opMode != null) {
            opMode.telemetry.addData("Arms Switch", "Arm0:(%b), Arm1 a:(%b), b:(%b)", zeroArm0.getState(), zeroArm1a.getState(), zeroArm1b.getState());
            opMode.telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            opMode.telemetry.update();
        }
    }

    public void gootoo(int pos0, int pos1) throws InterruptedException {
        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);
        while (arm0.isBusy()) {
            sleep(100);
            checkups();
        }
        while (arm1.isBusy()) {
            sleep(100);
            checkups();
        }
    }

    public void pleaseDo(Mode tmode) {
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
            setModeArm0(false);
            setModeArm1(false);
            switch (mode) {
                case HOME:
//                    if (arm0.getCurrentPosition() < 3000) {
//                        gootoo(3100, arm1.getCurrentPosition()); // 1
//                    }
                    gootoo(4100, 7202);
                    gootoo(4100, 4100);
                    gootoo(800, 200);
                    break;
                case PICK:
                    // peak up a cube and get back to drive position.
                    if (arm0.getCurrentPosition() < 3000 || arm1.getCurrentPosition() < 3000) {
                        gootoo(3100, 3100); // 1
                    }
                    rotateClamp(false);
                    gootoo(3000, 5400);  // 2
                    clamp(true);
                    if (driveClass != null)
                        driveClass.rollers(true);

                    //sleep(1000);
                    gootoo(2500, 5500); // 3
                    clamp(false);
                    sleep(1000);
                    gootoo(3680, 7202);  // 1
//                    sleep(2000);
//                    gootoo(4600, 6540 ); // 2
//                    sleep(2000);
//                    gootoo(3500,740);    // 3
//                    sleep(2000);
//                    gootoo(930,740);     // 4
                    break;

                case STRAIGHT:
                    gootoo(arm0.getCurrentPosition(), 9800);
                    gootoo(760, 9800);
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