package org.firstinspires.ftc.teamcode.ServoTestPractice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

@TeleOp(name="Servo Test SophieM")
public class ServoTest_SophieMansoor extends LinearOpMode {
    private Robot robot;

    double timePre = 0;
    double timeCurrent = 0;
    ElapsedTime timer = new ElapsedTime();

    public void initOpMode() throws IOException {
        this.robot = new Robot(this, timer);
    }

    public void runOpMode() {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        robot.initServosAuto();
        waitForStart();

        while(opModeIsActive()) {

            robot.getGamePadInputs();
            timeCurrent = timer.nanoseconds();

            //main claw arm
            if (robot.leftStickX > 0.5) {
                robot.control.modifyServo(robot.mainClawArm,0.005);
            } else if (robot.leftStickX > 0.1) {
                robot.control.modifyServo(robot.mainClawArm,0.001);
            } else if (robot.leftStickX > -0.1) {
                robot.control.modifyServo(robot.mainClawArm,0.0);
            } else if (robot.leftStickX > -0.5) {
                robot.control.modifyServo(robot.mainClawArm,-0.001);
            } else {
                robot.control.modifyServo(robot.mainClawArm,-0.005);
            }

            //main claw rotation
            if (robot.rightStickY > 0.5) {
                robot.control.modifyServo(robot.mainClawRotation,0.005);
            } else if (robot.rightStickY > 0.1) {
                robot.control.modifyServo(robot.mainClawRotation,0.001);
            } else if (robot.rightStickY > -0.1) {
                robot.control.modifyServo(robot.mainClawRotation,0.0);
            } else if (robot.rightStickY > -0.5) {
                robot.control.modifyServo(robot.mainClawRotation,-0.001);
            } else {
                robot.control.modifyServo(robot.mainClawRotation,-0.005);
            }

            //main claw
            if (robot.rightStickX > 0.5) {
                robot.control.modifyServo(robot.mainClaw,0.005);
            } else if (robot.rightStickX > 0.1) {
                robot.control.modifyServo(robot.mainClaw,0.001);
            } else if (robot.rightStickX > -0.1) {
                robot.control.modifyServo(robot.mainClaw,0.0);
            } else if (robot.rightStickX > -0.5) {
                robot.control.modifyServo(robot.mainClaw,-0.001);
            } else {
                robot.control.modifyServo(robot.mainClaw,-0.005);
            }

            //capstone arm
            if (robot.leftStickX2 > 0.5) {
                robot.control.modifyServo(robot.csArm,0.005);
            } else if (robot.leftStickX2 > 0.1) {
                robot.control.modifyServo(robot.csArm,0.001);
            } else if (robot.leftStickX2 > -0.1) {
                robot.control.modifyServo(robot.csArm,0.0);
            } else if (robot.leftStickX2 > -0.5) {
                robot.control.modifyServo(robot.csArm,-0.001);
            } else {
                robot.control.modifyServo(robot.csArm, -0.005);
            }

            //capstone claw
            if (robot.rightStickX2 > 0.5) {
                robot.control.modifyServo(robot.csClaw,0.005);
            } else if (robot.rightStickX2 > 0.1) {
                robot.control.modifyServo(robot.csClaw,0.001);
            } else if (robot.rightStickX2 > -0.1) {
                robot.control.modifyServo(robot.csClaw,0.0);
            } else if (robot.rightStickX2 > -0.5) {
                robot.control.modifyServo(robot.csClaw,-0.001);
            } else {
                robot.control.modifyServo(robot.csClaw,-0.005);
            }

            //foundation claw left
            if (robot.leftStickY2 > 0.5) {
                robot.control.modifyServo(robot.fClawL,0.005);
            } else if (robot.leftStickY2 > 0.1) {
                robot.control.modifyServo(robot.fClawL,0.001);
            } else if (robot.leftStickY2 > -0.1) {
                robot.control.modifyServo(robot.fClawL,0.0);
            } else if (robot.leftStickY2 > -0.5) {
                robot.control.modifyServo(robot.fClawL,-0.001);
            } else {
                robot.control.modifyServo(robot.fClawL,-0.005);
            }

            //foundation claw right
            if (robot.rightStickY2 > 0.5) {
                robot.control.modifyServo(robot.fClawR,0.005);
            } else if (robot.rightStickY2 > 0.1) {
                robot.control.modifyServo(robot.fClawR,0.001);
            } else if (robot.rightStickY2 > -0.1) {
                robot.control.modifyServo(robot.fClawR,0.0);
            } else if (robot.rightStickY2 > -0.5) {
                robot.control.modifyServo(robot.fClawR,-0.001);
            } else {
                robot.control.modifyServo(robot.fClawR,-0.005);
            }

            sleep(100);
        }

    }

}
