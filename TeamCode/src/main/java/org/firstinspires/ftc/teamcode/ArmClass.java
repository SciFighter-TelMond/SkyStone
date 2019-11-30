package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.annotation.Target;

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
        zeroArm1 = hardwareMap.get(DigitalChannel.class, "zero_arm1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1.setMode(DigitalChannel.Mode.INPUT);
    }

    public void reset(){
        arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm0.setTargetPosition(0);
        arm1.setTargetPosition(0);

        arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void begin() {
        reset();
        double power = 0.6;
        arm0.setPower(power);
        arm1.setPower(power);
    }

    public void moveArm0(double speed){
        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        }
        int ticks = arm0.getCurrentPosition() + (int) (100 * speed);
        if (speed != 0) {
            arm0.setTargetPosition(ticks);
        }
    }

    public void moveArm1(double speed){
 //       if (zeroArm1.getState() == false) {
 //           speed = Math.max(0, speed);
 //       }
        int ticks = arm1.getCurrentPosition() + (int) (100 * speed);
        if (speed != 0) {
            arm1.setTargetPosition(ticks);
        }
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
        try {

            /*
            Pos Name	Arm0	Arm1
            Pos 1 	3100	3100
            Pos 2 	2470	5080
            Pos 3	4700	5960
            Pos 4   500	    280

             */

            gooto(3100, 3100);
            sleep(2000);
            gooto(2470, 5080);
            sleep(2000);
            gooto(4700, 5960);
            sleep(2000);
            gooto(500, 280);

        } catch(InterruptedException e){ }
    }

    public void end(){
        interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
        arm0.setTargetPosition(arm0.getCurrentPosition());
        arm1.setTargetPosition(arm1.getCurrentPosition());
    }

    public int arm0getPos(){
        return arm0.getCurrentPosition();
    }

    public int arm1getPos(){
        return arm1.getCurrentPosition();
    }

}