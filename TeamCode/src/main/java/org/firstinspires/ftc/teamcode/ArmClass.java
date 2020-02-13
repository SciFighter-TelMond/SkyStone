package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmClass extends Thread {

    volatile private boolean stopFlag = true;

    volatile private DcMotorEx arm0 = null;
    volatile private DcMotorEx arm1 = null;
    volatile private DigitalChannel zeroArm0 = null;
    volatile private DigitalChannel zeroArm1 = null;

    volatile private Servo clamps = null;
    volatile private Servo clampsRotate = null;

    volatile private double power = 1;
    volatile private double speed_boost = 0.5;

    public enum Mode {IDLE, MANUAL, HOME, PICK, STRAIGHT, BUILD, DROP, SKY1, SKY2, SKY3}

    volatile private Mode mode = Mode.IDLE;

    // volatile private Toggle SpeedModeArm0 = new Toggle();
    volatile private int posArm0 = 0;
    volatile private int posArm1 = 0;

    volatile private OpMode opMode = null;
    volatile private DriveClass driveClass = null;

    static final int STAY = 999999;

    volatile private int currFloor = 0;

    public ArmClass(OpMode opMode, DriveClass drive) {
        this.setName("ArmClass");
        RobotLog.d("%s", this.getName());

        this.opMode = opMode;
        this.driveClass = drive;
    }

    public void init(HardwareMap hardwareMap) {
        RobotLog.d("ArmClass: Init");
        arm0 = hardwareMap.get(DcMotorEx.class, "arm0");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        zeroArm0 = hardwareMap.get(DigitalChannel.class, "zero_arm0");
        zeroArm1 = hardwareMap.get(DigitalChannel.class, "zero_arm1"); // TODO change names

        clamps = hardwareMap.get(Servo.class, "clamps");
        clampsRotate = hardwareMap.get(Servo.class, "clamps_rotate");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotorEx.Direction.FORWARD);
        arm1.setDirection(DcMotorEx.Direction.REVERSE);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1.setMode(DigitalChannel.Mode.INPUT);
        arm0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm0.setTargetPositionTolerance(10);
        arm1.setTargetPositionTolerance(10);


        //==============================
        // PIDF control

        RobotLog.d("Arm Velocity PID =======================================================");
        PIDFCoefficients pidf0 = arm0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf1 = arm1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RobotLog.d(pidf0.toString()); // p=2.000000 i=0.500000 d=0.000000 f=11.100006
        RobotLog.d(pidf1.toString()); // p=2.000000 i=0.500000 d=0.000000 f=11.100006

        // Arm 0
        pidf0.p = 4;
        pidf0.i = 3;
        pidf0.d = 0.1;
        arm0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf0);
        arm0.setPositionPIDFCoefficients(7);

        // Arm 1
        pidf1.p = 4;
        pidf1.i = 3;
        pidf1.d = 0.1;
        arm1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf1);
        arm1.setPositionPIDFCoefficients(7);

        RobotLog.d(pidf0.toString());
        RobotLog.d(pidf1.toString());

        RobotLog.d("ArmClass: /init ");
    }

    public void begin() {
        RobotLog.d("ArmClass: begin");
        reset();
        mode = Mode.MANUAL;
        stopFlag = false;
    }

    public void reset() {
        RobotLog.d("ArmClass: reset");
        arm0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setArmDriveMode(true);
    }

    public void clamp(boolean open) {
        if (open) {
            clamps.setPosition(0);
        } else {
            clamps.setPosition(1);
        }
    }

    public void rotateClamp(boolean rot) {
        if (rot) {
            clampsRotate.setPosition(1);
        } else {
            clampsRotate.setPosition(0);
        }
    }

    public void setArmDriveMode(boolean drive) {
        if (drive) {
            arm0.setPower(0);
            arm0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm1.setPower(0);
            arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            arm0.setTargetPosition(arm0.getCurrentPosition());
            arm0.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm0.setPower(power);
            arm1.setTargetPosition(arm1.getCurrentPosition());
            arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm1.setPower(power / 2);
        }
    }

    public void setBoost(double boost) {
        speed_boost = 0.4 + boost * 0.6;
    }

    public void moveArm0(double speed) {
        if (mode != Mode.MANUAL) {
//            RobotLog.d("Arm0 move interrupt");
//            this.interrupt();
            return;
        }

        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        }

        arm0.setPower(speed * speed_boost);
    }

    public void moveArm1(double speed) {
        if (mode != Mode.MANUAL) {
//            RobotLog.d("Arm1 move interrupt");
//            this.interrupt();
            return;
        }
        if (zeroArm1.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (speed > 0 && arm1.getCurrentPosition() > 950 - speed * 20) { // TODO:
            speed = 0;
        }
        arm1.setPower(speed * speed_boost / 2);
    }

    public void checkups() {
        if (zeroArm0.getState() == false && arm0.getTargetPosition() < arm0.getCurrentPosition()) {
            RobotLog.d("Arm0 checkups hit at: %d", arm0.getCurrentPosition());
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }
        if (zeroArm1.getState() == false && arm1.getTargetPosition() < arm1.getCurrentPosition()) {
            RobotLog.d("Arm1 checkups hit at: %d", arm1.getCurrentPosition());
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }
        //TODO: checkout why the safty bray works when not supposed to.
        int diff0 = Math.abs(arm0.getCurrentPosition() - posArm0);
        if (diff0 <= 5) {
            arm0.setTargetPosition(arm0.getCurrentPosition());
            opMode.telemetry.addData("BRAKE", "Arm0 diff %d", diff0);
        }
        //
        int diff1 = Math.abs(arm1.getCurrentPosition() - posArm1);
        if (diff1 <= 5) {
            arm1.setTargetPosition(arm1.getCurrentPosition());
            opMode.telemetry.addData("BRAKE", "Arm1 diff %d", diff1);
        }
//
//        posArm0 = arm0.getCurrentPosition();
//        posArm1 = arm1.getCurrentPosition();
//

//        if (opMode != null) {
//            opMode.telemetry.addData("Arms Switch", "Arm0:(%b), Arm1:(%b)", zeroArm0.getState(), zeroArm1.getState());
//            opMode.telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
//            opMode.telemetry.update();
//        }
    }

    public void gootoo(int pos0, int pos1) throws InterruptedException {
        if (pos0 == STAY) pos0 = arm0.getCurrentPosition();
        if (pos1 == STAY) pos1 = arm1.getCurrentPosition();

        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        //RobotLog.d("Arm goto start to: Arm0:(%d)->(%d), Arm1(%d)->(%d)  ", arm0.getCurrentPosition(), arm0.getTargetPosition(), arm1.getCurrentPosition(), arm1.getTargetPosition());
        while (arm0.isBusy() || arm1.isBusy()) {
            sleep(50);
            //RobotLog.d("Arm goto run at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            checkups();
        }

        //RobotLog.d("Arm goto complete at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
    }

    public void gootoo(int pos0, int pos1, double timeout) throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        if (pos0 == STAY) pos0 = arm0.getCurrentPosition();
        if (pos1 == STAY) pos1 = arm1.getCurrentPosition();

        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        //RobotLog.d("Arm goto start to: Arm0:(%d)->(%d), Arm1(%d)->(%d)  ", arm0.getCurrentPosition(), arm0.getTargetPosition(), arm1.getCurrentPosition(), arm1.getTargetPosition());
        while ((arm0.isBusy() || arm1.isBusy()) && runTime.seconds() < timeout) {
            sleep(50);
            //RobotLog.d("Arm goto run at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            checkups();
        }

        //RobotLog.d("Arm goto complete at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
    }

    public void pleaseDo(Mode newMode) {
        if (mode != Mode.MANUAL && mode != Mode.IDLE) {
            RobotLog.d("Arm pleaseDo busy doing %s", mode.toString());
            // interrupt();
            return;
        }
        RobotLog.d("Arm pleaseDo: %s", newMode.toString());
        mode = newMode;
        if (newMode != Mode.MANUAL || newMode != Mode.IDLE) {
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
            power = 0.8;
            setArmDriveMode(false);
            switch (mode) {
                case HOME:
                    RobotLog.d("Arm do: HOME");
                    gootoo(300, STAY);
                    driveClass.rollers(true);
                    sleep(500);
                    gootoo(-500, -500);
                    reset();
                    driveClass.rollers(false);
                    RobotLog.d("Arm do: HOME/");
                    break;

                case PICK:
                    // peak up a cube and get back to drive position.
                    double rollersState = 0;
                    RobotLog.d("Arm do: PICK");
                    rotateClamp(false);

                    RobotLog.d("Arm do: PICK - Open clamp");
                    clamp(true);
                    RobotLog.d("Arm do: PICK - Open Rollers");
                    driveClass.rollers(true);

                    RobotLog.d("Arm do: PICK - Go above");
                    gootoo(500, 0);
                    RobotLog.d("Arm do: PICK - Go down");
                    gootoo(-200, -200);
                    clamp(false);
                    RobotLog.d("Arm do: before sleep");
                    sleep(700);
                    RobotLog.d("Arm do: after sleep");
                    gootoo(400, 0);
                    driveClass.rollers(false);
                    RobotLog.d("Arm do: PICK/");
                    break;

                case BUILD:
                    RobotLog.d("Arm do: BUILD");
                    if (arm0.getTargetPosition() < 100)
                        gootoo(300, 0);

                    switch (currFloor) {
                        case 0:
                        case 1:
                            RobotLog.d("Arm do: BUILD - floor 1");
                            gootoo(830, STAY);
                            gootoo(830, 560);
                            gootoo(225, 555);
                            break;
                        case 2:
                            RobotLog.d("Arm do: BUILD - floor 2");
                            gootoo(930, STAY);
                            gootoo(930, 600);
                            gootoo(700, 730);
                            break;
                        case 3:
                            RobotLog.d("Arm do: BUILD - floor 3");
                            gootoo(1260, 80);
                            gootoo(1030, 760);
                            break;
                        case 4:
                            RobotLog.d("Arm do: BUILD - floor 4");
                            gootoo(1630, 180);
                            gootoo(1280, 870);
                            break;
                        case 5:
                            RobotLog.d("Arm do: BUILD - floor 5");
                            gootoo(1870, 540);
                            gootoo(1485, 900);
                            break;
                        case 6:
                            RobotLog.d("Arm do: BUILD - floor 6");
                            gootoo(3000, 510);
                            gootoo(3700, 540);
                            break;
                        case 7:
                            RobotLog.d("Arm do: BUILD - floor 7");
                            gootoo(3000, 800);
                            gootoo(3615, 820);
                            break;
                        case 8:
                            RobotLog.d("Arm do: BUILD - floor 8");
                            gootoo(3000, 900);
                            break;
                        default:
                            RobotLog.d("Arm do: BUILD - NO floor");
                            break;
                    }

                    RobotLog.d("Arm do: BUILD/");
                    break;

                case SKY1: // open arm to start position
                    gootoo(515, 0);
                    rotateClamp(true);
                    clamp(true);
                    gootoo(515, 360);
                    break;

                case SKY2: // after catch move arm back
                    gootoo(400, 240);
                    break;

                case SKY3: // get ready to catch
                    gootoo(515, 360);
                    break;

                default:
                    break;
            }
        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        } catch (Exception e) {
            RobotLog.logStackTrace(e);
        }
        power = 1;
        setArmDriveMode(true);
        RobotLog.d("Arm Thread complete");
        mode = Mode.MANUAL;
    }

    public void end() {
        RobotLog.d("ArmClass:End()");
        stopFlag = true;
        interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
        arm0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mode = Mode.IDLE;
    }

    public void resumePower() {
        RobotLog.d("ArmClass: Resume");
        setArmDriveMode(true);
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
        return zeroArm1.getState();
    }

    public void floorPlus() {
        if (currFloor >= 8) {
            currFloor = 8;
        } else {
            currFloor++;
        }
    }

    public void floorMinus() {
        if (currFloor <= 1) {
            currFloor = 1;
        } else {
            currFloor--;
        }
    }
}