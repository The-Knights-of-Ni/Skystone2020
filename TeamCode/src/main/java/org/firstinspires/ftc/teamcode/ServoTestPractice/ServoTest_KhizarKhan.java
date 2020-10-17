package org.firstinspires.ftc.teamcode.ServoTestPractice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

@TeleOp(name="Servo Test KhizarK")
public class ServoTest_KhizarKhan extends LinearOpMode{
    private Robot robot;

    double PreTime = 0;
    double CurrentTime = 0;
    ElapsedTime timer = new ElapsedTime();


    public void initOpMode() throws IOException {
        this.robot = new Robot (this, timer);

    }

    public void runOpMode() {
        try {
            initOpMode();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        robot.initServosAuto();
        telemetry.clearAll();

        waitForStart(); //waits for start

        while (opModeIsActive()) {
            //Get gamepad inputs
            robot.getGamePadInputs();

            //Get the current time
            CurrentTime = timer.nanoseconds();


            //main claw arm servo test
            if (robot.leftStickX > 0.5) //controls main claw ARM itself
                robot.control.modifyServo(robot.mainClawArm, 0.005);
            else if (robot.leftStickX > 0.1)
                robot.control.modifyServo(robot.mainClawArm, 0.001);
            else if (robot.leftStickX > -0.1) //do nothing
                robot.control.modifyServo(robot.mainClawArm, -0.0);
            else if  (robot.leftStickX > -0.5)
                robot.control.modifyServo(robot.mainClawArm, -0.001);
            else
                robot.control.modifyServo(robot.mainClawArm, -0.005);

            if (robot.rightStickY > 0.5) //these control main claw arm rotation
                robot.control.modifyServo(robot.mainClawRotation, 0.005);
            else if (robot.rightStickY > 0.1)
                robot.control.modifyServo(robot.mainClawRotation, 0.001);
            else if (robot.rightStickY > -0.1) //do nothing
                robot.control.modifyServo(robot.mainClawRotation, -0.0);
            else if  (robot.rightStickY > -0.5)
                robot.control.modifyServo(robot.mainClawRotation, -0.001);
            else
                robot.control.modifyServo(robot.mainClawRotation, -0.005);

            if (robot.rightStickX > 0.5) //controls main claw itself
                robot.control.modifyServo(robot.mainClaw,0.005);
            else if (robot.rightStickX > 0.1)
                robot.control.modifyServo(robot.mainClaw,0.001);
            else if (robot.rightStickX > -0.1)
                robot.control.modifyServo(robot.mainClaw,0.0);
            else if (robot.rightStickX > -0.5)
                robot.control.modifyServo(robot.mainClaw,-0.001);
            else
                robot.control.modifyServo(robot.mainClaw,-0.005);

        //EVERYTHING FROM HERE IS GAMEPAD 2
            //controls servos for capstone arms
            if (robot.leftStickX2 > 0.5)  //controls capstone arm
                robot.control.modifyServo(robot.csArm,0.005);
            else if (robot.leftStickX2 > 0.1)
                robot.control.modifyServo(robot.csArm,0.001);
            else if (robot.leftStickX2 > -0.1)
                robot.control.modifyServo(robot.csArm,0.0);
            else if (robot.leftStickX2 > -0.5)
                robot.control.modifyServo(robot.csArm,-0.001);
            else
                robot.control.modifyServo(robot.csArm,-0.005);

            if (robot.rightStickX2 > 0.5)  //controls capstone claw
                robot.control.modifyServo(robot.csClaw,0.005);
            else if (robot.rightStickX2 > 0.1)
                robot.control.modifyServo(robot.csClaw,0.001);
            else if (robot.rightStickX2 > -0.1)
                robot.control.modifyServo(robot.csClaw,0.0);
            else if (robot.rightStickX2 > -0.5)
                robot.control.modifyServo(robot.csClaw,-0.001);
            else
                robot.control.modifyServo(robot.csClaw,-0.005);

            //control servos for foundation claw
            if (robot.leftStickY2 > 0.5)  //controls left foundation claw
                robot.control.modifyServo(robot.fClawL,0.005);
            else if (robot.leftStickY2 > 0.1)
                robot.control.modifyServo(robot.fClawL,0.001);
            else if (robot.leftStickY2 > -0.1)
                robot.control.modifyServo(robot.fClawL,0.0);
            else if (robot.leftStickY2 > -0.5)
                robot.control.modifyServo(robot.fClawL,-0.001);
            else
                robot.control.modifyServo(robot.fClawL,-0.005);

            if (robot.rightStickY2 > 0.5)  //controls right foundation claw
                robot.control.modifyServo(robot.fClawR,0.005);
            else if (robot.rightStickY2 > 0.1)
                robot.control.modifyServo(robot.fClawR,0.001);
            else if (robot.rightStickY2 > -0.1)
                robot.control.modifyServo(robot.fClawR,0.0);
            else if (robot.rightStickY2 > -0.5)
                robot.control.modifyServo(robot.fClawR,-0.001);
            else
                robot.control.modifyServo(robot.fClawR,-0.005);

            sleep(100);

        }





    }


}
