package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClassTest extends Thread {

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel zeroSwitch0 = null;
    private DigitalChannel zeroSwitch1 = null;

    public enum Commands { MANUAL, SEARCH_0, GOTO_0, TACK_BLOCK, BUILD_BLOCK_0, BUILD_BLOCK_1, BUILD_BLOCK_2, BUILD_BLOCK_3, BUILD_BLOCK_4 };
    private Commands currentCommand = Commands.MANUAL;

    public void setCommand( Commands command ) {
        this.currentCommand = command;
        start();
    }

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

    public void begine() {
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setTargetPosition(0);
        arm1.setTargetPosition(0);
        arm0.setPower(1);
        arm1.setPower(1);
        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TODO: if not zero switch touched - search for the zero position.
    }

    public void end() {
        this.interrupt();
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setPower(0);
        arm1.setPower(0);
    }

    public void moveSteps( int steps0, int steps1 ) {
        if (currentCommand != Commands.MANUAL) {
            interrupt();
        }
        int pos0 = arm0.getCurrentPosition() + steps0;
        arm0.setTargetPosition(pos0);

        int pos1 = arm1.getCurrentPosition() + steps1;
        arm1.setTargetPosition(pos1);

        checkZeroSwitch();
    }

    private void checkZeroSwitch() {

        if (zeroSwitch0.getState() == false) {
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }

        if (zeroSwitch1.getState() == false) {
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }
    }

    private void goToPosition(int pos0, int pos1) throws InterruptedException {
        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( arm0.isBusy() ) {
            sleep(10);
            checkZeroSwitch();
        }
        while ( arm1.isBusy() ) {
            sleep(10);
            checkZeroSwitch();
        }
    }

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
}
