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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AUTO - TES2T", group="SciFighters")// moving the blue foundation. you are in the blue team.
//@Disabled
public class Auto_TEST2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveClass        robot = new DriveClass(this);   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.init_GyroIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();

        DriveClass.Alliance team = DriveClass.Alliance.RED;
        double mul = 1;

        runtime.reset();


        robot.rotateTo(180 * mul,1,3, 3);      // rotate 180
        // straight(0.4, Direction.REVERSE,1,3, 180, true);    // drive to foundation
        robot.catchFoundation(180);
        // arm.pleaseDo(ArmClass.Mode.SKY5_DROP_BACK);                          // drop the stone backwards and HOME the arm while moving the foundation.

        // robot.straight(0.3, DriveClass.Direction.FORWARD,1,3, 180, false);

        // Rotate the foundation to the building zone.
        robot.rotate(0.05 * mul, DriveClass.Direction.RIGHT,1,0.5);       // rotate 15c - just a little to gain angle.

        double angle = 180 + 20 * mul ;
        robot.straight(1, DriveClass.Direction.FORWARD,1,3, angle, false);  // move forward half way - move foundation from the wal.

        double bridgeDirection =  - 90 * mul ; // bridge direction
        robot.rotateTo(bridgeDirection,1,3, 5);      // rotate foundation to building zone.
        robot.hooksUp();                                                      // release foundation.
        //sleep(150);                                  // wait for hooks to open
        robot.strafe(0.3, DriveClass.Direction.RIGHT,1,1,bridgeDirection);

        // Parking - drive FORWARD towards bridge line to park - search the line and park.
        robot.straight(1.1, DriveClass.Direction.FORWARD,1,3, bridgeDirection, false);       // move forward toward bridge.
        timer.reset();
        robot.drive(0.5, 0, 0, 0, 0); // drive slowly until it sees the line.
        while (!robot.isLine(team) && timer.seconds() < 1.5  && opModeIsActive()) {
            sleep(1);
        }
        stop();
        // waist time until the end to let the arm HOME finish.
        while (opModeIsActive()) {
            sleep(10);
        }

        robot.stop();
        //sleep(1000);
    }
}
