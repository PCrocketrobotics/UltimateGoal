
package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.*;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControlled")
public class DriverControlled extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot  = new Robot(this);

    @Override
    public void runOpMode() {
        // Initialize all the Robot Parts.
        // First Init the Drive Train by setting up the DriveTrain Motors and Sensors.
        telemetry.addData("Robot Initialized ... ", "START");    //
        telemetry.update();
        robot.getDriveTrain().init();
        robot.getWobbleGripper().init();
        robot.getRingControl().init();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot Initialized ... ", "DONE");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // clear telemetry to clean up screen
        telemetry.clearAll();
        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
                robot.getDriveTrain().DriverControlled_Drive();
                robot.getRingControl().DriverControlledRingIntake();
                robot.getRingControl().DriverControlledRingShooter();
                robot.getWobbleGripper().DriverControlled_WobbleServo();
                telemetry.update();
            // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }

}

