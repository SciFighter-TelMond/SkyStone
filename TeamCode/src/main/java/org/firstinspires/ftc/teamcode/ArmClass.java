package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    public enum Mode {IDLE, MANUAL, HOME, PICK, STRAIGHT, BUILD, DROP}

    volatile private Mode mode = Mode.IDLE;

    // volatile private Toggle SpeedModeArm0 = new Toggle();
    volatile private int posArm0 = 0;
    volatile private int posArm1 = 0;

    volatile private OpMode opMode = null;
    volatile private DriveClass driveClass = null;

    static final int STAY = 999999;

    volatile private int currFloor = 1;

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
        setModeArm(true);
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
            clampsRotate.setPosition(1);
        } else {
            clampsRotate.setPosition(0);
        }
    }

    private void setModeArm(boolean drive) {
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
        if (zeroArm1.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (speed > 0 && arm1.getCurrentPosition() > 950 - speed * 20) { // TODO:
            speed = 0;
        }
        arm1.setPower(speed * speed_boost / 2);
    }

    public void checkups() {
        if (zeroArm0.getState() == false) {
            arm0.setTargetPosition(Math.max(arm0.getCurrentPosition(), arm0.getTargetPosition()));
        }
        if (zeroArm1.getState() == false) {
            arm1.setTargetPosition(Math.max(arm1.getCurrentPosition(), arm1.getTargetPosition()));
        }

// TODO: checkout why the safty bray works when not supposed to.
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

        posArm0 = arm0.getCurrentPosition();
        posArm1 = arm1.getCurrentPosition();


        if (opMode != null) {
            opMode.telemetry.addData("Arms Switch", "Arm0:(%b), Arm1:(%b)", zeroArm0.getState(), zeroArm1.getState());
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
        if (mode != Mode.MANUAL && mode != Mode.IDLE) {
            interrupt();
            return;
        }
        RobotLog.d("Arm pleaseDo");
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
            power = 0.4; // TODO: remove
            setModeArm(false);
            switch (mode) {
                case HOME:
                    RobotLog.d("Arm do: HOME");
                    rotateClamp(true);
                    gootoo(0, 0);
                    RobotLog.d("Arm do: HOME/");
                    break;

                case PICK:
                    // peak up a cube and get back to drive position.
                    double rollersState = 0;
                    RobotLog.d("Arm do: PICK");
                    rotateClamp(false);
                    if (driveClass != null) {
                        RobotLog.d("Arm do: PICK - Rollers");
                        rollersState = driveClass.getRollersPower();
                        driveClass.rollersRunIn();
                        driveClass.rollers(true);
                    }
                    gootoo(500, 0);
                    clamp(true);
                    gootoo(-200, -200);
                    clamp(false);
                    if (driveClass != null) {
                        if (rollersState == 0)
                            driveClass.rollersStop();
                    }
                    sleep(1000);
                    gootoo(400, 0);
                    RobotLog.d("Arm do: PICK/");
                    break;

                case BUILD:
                    RobotLog.d("Arm do: BUILD");
                    if (arm0.getTargetPosition() < 100)
                        gootoo(300, 0);

                    switch (currFloor) {
                        case 1:
                            gootoo(305, 435);
                            break;
                        case 2:
                            gootoo(720, 490);
                            break;
                        case 3:
                            gootoo(1025, 530);
                            break;
                        case 4:
                            gootoo(1225, 855);
                            break;
                        case 5:
                            gootoo(1535, 835);
                            break;
                        case 6:
//                            gootoo(600, 600);
                            break;
                        case 7:
//                            gootoo(700, 700);
                            break;
                        case 8:
//                            gootoo(800, 800);
                            break;
                        default:
                            break;
                    }

                    RobotLog.d("Arm do: BUILD/");
                    break;

                default:
                    break;
            }
        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        } catch (Exception e) {
            RobotLog.logStackTrace(e);
        }
        power = 1; // TODO: remove
        setModeArm(true);
        mode = Mode.MANUAL;
    }

    public void end() {
        RobotLog.d("ArmClass: begin");
        stopFlag = true;
        interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
        arm0.setTargetPosition(arm0.getCurrentPosition());
        arm1.setTargetPosition(arm1.getCurrentPosition());
        mode = Mode.IDLE;
    }

    public void resumePower() {
        RobotLog.d("ArmClass: Resume");
        setModeArm(true);
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

    public void floorPlus(){
        if (currFloor >= 8){
            currFloor = 8;
        }
        else{
            currFloor++;
        }
    }

    public void floorMinus(){
        if (currFloor <= 1){
            currFloor = 1;
        }
        else{
            currFloor--;
        }
    }
}