package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClass extends Thread {

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DigitalChannel zeroArm0 = null;
    private DigitalChannel zeroArm1 = null;

    ArmClass() {

    }

    public void init(HardwareMap hardwareMap) {
        arm0 = hardwareMap.get(DcMotor.class, "arm0");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        zeroArm0 = hardwareMap.get(DigitalChannel.class, "zero_arm0");
        zeroArm1 = hardwareMap.get(DigitalChannel.class, "zero_arm0");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1.setMode(DigitalChannel.Mode.INPUT);
    }

    public void begin() {
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 0.6;
        arm0.setPower(power);
        arm1.setPower(power);
    }

    public void gooto(int pos0,int pos1) throws InterruptedException {
        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);
        while(arm0.isBusy()){
            sleep(10);
        }
        while(arm1.isBusy()){
            sleep(10);
        }
    }

    public void run() {

    }
    public void end(){
        double power = 0.0;
        arm0.setPower(power);
        arm1.setPower(power);
    }
}