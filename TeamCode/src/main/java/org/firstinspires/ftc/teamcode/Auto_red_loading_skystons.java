/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 */

@Autonomous(name="Red Sky-Stone", group="SciFighterd")// moving the blue foundation. you are in the blue team.
//@Disabled
public class Auto_red_loading_skystons extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveClass robot = new DriveClass(this);   // Use a Pushbot's hardware
    private ArmClass arm = new ArmClass(this, robot);

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        arm.init(hardwareMap);
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        arm.begin();
        arm.setModeArm(false);

        runtime.reset();

        try {

            //robot.side(0.6, DriveClass.Direction.RIGHT, 1, 3);
            arm.gootoo(250, 0);
            arm.rotateClamp(true);
            arm.clamp(true);
            robot.straight(1, DriveClass.Direction.FORWARD, 0.7, 4);

            // drive straight close to stones
            robot.drive(0.3, 0, 0, 0, 0);
            timer.reset();
            while (robot.getSensorDistanceLeft() > 3 && opModeIsActive() && timer.seconds() < 5) {
                telemetry.addData("Dist  ", robot.getSensorDistanceLeft());
                telemetry.update();
                sleep(10);
            }
            robot.stop();

            // drive LEFT : search for SkyStone
            robot.drive(0, -0.8, 0, 0, 0);
            timer.reset();
            while (!robot.isSkyStoneLeft() && opModeIsActive() && timer.seconds()<10) {
                telemetry.update();
                sleep(10);
            }

            if(timer.seconds()<1) {
                robot.side(0.20, DriveClass.Direction.LEFT, 0.4, 2);
            }
            else
            {
            // drive LEFT one block
            robot.side(0.32, DriveClass.Direction.LEFT, 0.4, 2);
            robot.stop();}

            // catch STONE
            arm.gootoo(90, 430);
            arm.clamp(false);
            sleep(500);
            arm.gootoo(300, 300);

            // drive backwards
            robot.straight(0.3, DriveClass.Direction.REVERSE,0.5,1);

            // drive RIGHT : search for red line (under the brig)
            robot.drive(0,0.7,0,0,0);
            timer.reset();
            while (!robot.isRed() && opModeIsActive() && timer.seconds()<8) {
                // telemetry.update();
                sleep(1);
            }
            robot.stop();

            // slide RIGHT to put stone
            robot.side(0.7, DriveClass.Direction.RIGHT,0.5,2);
            robot.straight(0.2, DriveClass.Direction.FORWARD,0.5,1);
            robot.stop();

            arm.clamp(true);

            arm.gootoo(300,0);

            arm.clamp(false);

            //arm.gootoo(,0);


            // drive LEFT : beck to line.
            robot.drive(0,-0.5,0,0,0);
            timer.reset();
            while (!robot.isRed() && opModeIsActive() && timer.seconds()<2) {
                //telemetry.update();
                sleep(1);
            }

            robot.stop();
            while (opModeIsActive() && (runtime.seconds() < 30)) {
                robot.isRed();
                telemetry.update();
            }

            robot.stop();
        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        }

        robot.end();
        arm.end();
    }

}