package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmClass extends Thread {

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel zeroArm0 = null;
    private DigitalChannel zeroArm1 = null;

    private Servo clamps = null;
    private Servo clampsRotate = null;

    private int manualPos0 = 0;
    private int manualPos1 = 0;

    private double power         = 1;
    private double motor_speed_k = 300;
    private double speed_boost   = 0.5;

    public enum Mode {IDLE, MANUAL, PICK, BUILD, DORP}

    private Mode mode = Mode.IDLE;

    private boolean arm0Move = false;
    private boolean arm1Move = false;

    private OpMode opMode = null;
    private DriveClass driveClass = null;

    public ArmClass(OpMode opMode, DriveClass drive)
    {
        this.setName("ArmClass");
        RobotLog.d("%s", this.getName());

        this.opMode = opMode ;
        this.driveClass = drive ;
    }

    public void init(HardwareMap hardwareMap) {
        arm0     = hardwareMap.get(DcMotor.class, "arm0");
        arm1     = hardwareMap.get(DcMotor.class, "arm1");
        zeroArm0 = hardwareMap.get(DigitalChannel.class, "zero_arm0");
        zeroArm1 = hardwareMap.get(DigitalChannel.class, "zero_arm1");
        clamps   = hardwareMap.get(Servo.class, "clamps");
        clampsRotate = hardwareMap.get(Servo.class, "clamps_rotate");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1.setMode(DigitalChannel.Mode.INPUT);
    }

    public void reset() {
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm0.setTargetPosition(0);
        arm1.setTargetPosition(0);

        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void begin() {
        reset();
        arm0.setPower(power);
        arm1.setPower(power);
        mode = Mode.MANUAL;
    }

    public void setBoost( double boost ) {
        speed_boost = 0.5 + boost * 0.5;
    }

    public void moveArm0(double speed) {
        if (mode != Mode.MANUAL)
            return;

        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (arm0Move || speed !=0) {
            double k = motor_speed_k * speed_boost;
            int ticks = arm0.getCurrentPosition() + (int) (k * speed);
            arm0.setTargetPosition(ticks);
            arm0Move = speed != 0;
        }
    }

    public void moveArm1(double speed) {
        if (mode != Mode.MANUAL)
            return;
        //       if (zeroArm1.getState() == false) {
        //           speed = Math.max(0, speed);
        //       }
        if (arm1Move || speed !=0) {
            double k = motor_speed_k * speed_boost;
            int ticks = arm1.getCurrentPosition() + (int) (k * speed);
            arm1.setTargetPosition(ticks);
            arm1Move = speed != 0;
        }
    }

    public void checkups() {
        if (zeroArm0.getState() == false) {
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }
//        if (zeroArm1.getState() == false) {
//            arm1.setTargetPosition(arm1.getCurrentPosition());
//        }

        if (opMode != null) {
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
            switch (mode) {
                case PICK:
                    // peak up a cube and get back to drive position.
                    gootoo(3100, 3100); // 1
                    gootoo(3100,5400);  // 2
                    rotateClamp(false);
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

                case BUILD:
                    // build
                    break;

                case DORP:
                    break;

                default: break;
            }
        }
        catch (InterruptedException e) { RobotLog.d("Arm Thread interrupted!"); }
        catch (Exception e) { RobotLog.logStackTrace(e); }

        mode = Mode.MANUAL;
    }

    public void end() {
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
    }

    public int getArm0Pos() {
        return arm0.getCurrentPosition();
    }

    public int getArm1Pos() {
        return arm1.getCurrentPosition();
    }

    public void clamp(boolean open) {
        if (open) {
            clamps.setPosition(1);
        } else {
            clamps.setPosition(0);
        }
    }

    public void rotateClamp(boolean y) {
        if (y) {
            clampsRotate.setPosition(1);
        } else {
            clampsRotate.setPosition(0);
        }
    }
}