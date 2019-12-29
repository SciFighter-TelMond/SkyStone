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
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="Blue Loading 2", group="SciFighterd")// moving the blue foundation. you are in the blue team.
//@Disabled
public class Auto_blue_loading2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveClass         robot   = new DriveClass(this);   // Use a Pushbot's hardware
    private ArmClass    arm = new ArmClass(this, robot);
    private ElapsedTime     runtime = new ElapsedTime();

    private Toggle clamps_rotate    = new Toggle();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        arm.init(hardwareMap);
        arm.begin();
        arm.reset();
        arm.rotateClamp(clamps_rotate.getState());// turn clamps to
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.straight(1.5, DriveClass.Direction.FORWARD, 0.4, 5);
        // find special cube
        //and stop in front of
        try {
            arm.gootoo(300, 300);
            arm.gootoo(100, 300);
            arm.clamp(false);
            arm.gootoo(300, 200);
            robot.side(3, DriveClass.Direction.LEFT, 0.5, 5);


        }catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        }



        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        runtime.reset();
        /*starting point: building area, the bridge is on the left,
         * the front of the robot is to the wall, the hooks directed to the middle of the field.
         * please start close to the building site*/
        // Step 1:  Drive forward (actually back) and a bit to the right (actually to the left)
//        robot.rollers(true);
//        robot.rollersRunIn();
//        robot.side(0.6, DriveClass.Direction.RIGHT, 1, 3);
//        robot.straight(1.7, DriveClass.Direction.FORWARD, 0.4, 5);
//        robot.straight(0.2, DriveClass.Direction.REVERSE, 0.4, 5);
//        robot.straight(0.2, DriveClass.Direction.FORWARD, 0.4, 5);
//        robot.rollers(false);
//        robot.rollersRunIn();
//        robot.rotate(0.3, DriveClass.Direction.LEFT, 0.5, 5);
//        robot.straight(2.2, DriveClass.Direction.FORWARD, 0.4, 5);
//        robot.rollersRunOut();
//        robot.straight(0.4, DriveClass.Direction.REVERSE, 0.4, 5);

        sleep(500);


        // Step 2: should be in front of the foundation, hooks down

        sleep(200);

              while (opModeIsActive() && (runtime.seconds() < 30)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 6:  stop
        robot.stop();
        //sleep(1000);
    }
}
