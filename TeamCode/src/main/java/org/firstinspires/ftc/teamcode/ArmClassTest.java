package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClassTest extends Thread {

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel zeroSwitch0 = null;
    private DigitalChannel zeroSwitch1 = null;

    public enum Commands { IDLE, MANUAL, SEARCH_0, GOTO_0, TACK_BLOCK, BUILD_BLOCK_0, BUILD_BLOCK_1, BUILD_BLOCK_2, BUILD_BLOCK_3, BUILD_BLOCK_4 };
    private Commands currentCommand = Commands.MANUAL;

    private double power = 0.7;

    // ====================================================================================
    public void setCommand( Commands command ) {
        this.currentCommand = command;
        start();
    }

    // ====================================================================================
    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        arm0 = hardwareMap.get(DcMotor.class, "arm0");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        zeroSwitch0 = hardwareMap.get(DigitalChannel.class, "start_game0");
        zeroSwitch1 = hardwareMap.get(DigitalChannel.class, "start_game1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        zeroSwitch0.setMode(DigitalChannel.Mode.INPUT);
        zeroSwitch1.setMode(DigitalChannel.Mode.INPUT);

        arm0.setPower(0);
        arm1.setPower(0);
    }

    // ====================================================================================
    public void setPower(double power) {
        this.power = power;
        if (currentCommand != Commands.IDLE) {
            arm0.setPower(power);
            arm1.setPower(power);
        }
    }

    // ====================================================================================
    public void reset() {
        this.interrupt();
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setTargetPosition(0);
        arm1.setTargetPosition(0);
        arm0.setPower(power);
        arm1.setPower(power);
        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ====================================================================================
    public void begine() {
        reset();
        // TODO: if not zero switch touched - search for the zero position.
        currentCommand = Commands.MANUAL;
    }

    // ====================================================================================
    private void checkSwitch() {
        if (zeroSwitch0.getState() == false) {
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }

        if (zeroSwitch1.getState() == false) {
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }
    }

    // ====================================================================================
    public void moveArm0(float speed) {
        if (zeroSwitch0.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (speed != 0) {
            int tiks = arm0.getCurrentPosition() + (int) (10 * speed);
            arm0.setTargetPosition(tiks);
        }
    }

    // ====================================================================================
    public void moveArm1(float speed) {
        if (zeroSwitch1.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (speed != 0) {
            int tiks = arm1.getCurrentPosition() + (int) (10 * speed);
            arm1.setTargetPosition(tiks);
        }
    }

    // ====================================================================================
    private void goToPosition(int pos0, int pos1) throws InterruptedException {
        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        while ( arm0.isBusy() ) {
            sleep(10);
            checkSwitch();
        }
        while ( arm1.isBusy() ) {
            sleep(10);
            checkSwitch();
        }
    }

    // ====================================================================================
    @Override
    public void run() {
        try {
            switch (currentCommand) {

                 case GOTO_0:
                    if (arm1.getCurrentPosition() < 0)
                        goToPosition(1000, arm1.getCurrentPosition());
                    goToPosition(0, 0);
                    break;

                default:
                    break;
            }
        } catch (InterruptedException e) { }

        currentCommand = Commands.MANUAL;
    }

    // ====================================================================================
    public void end() {
        this.interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
