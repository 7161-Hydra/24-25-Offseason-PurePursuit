package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class AdvancedTeleOp extends LinearOpMode {
    Vector2d currentVector;
    static double speed;
    double targetHeading;
    Pose2d pose;
    double angle; //the angle variable the robot lock onto after the touchpad had been released after button press

    //used to lock the heading after timeout ms from last right stick input
    ElapsedTime timeFromLastInput = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if the pose is gotten from the auto(poseStorage), or pre
        //drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(-30, 0, Math.toRadians(0)));
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        speed = .5;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        targetHeading = drive.getPoseEstimate().getHeading();



        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.ps || gamepad1.right_stick_button) {
                drive.setPoseEstimate(new Pose2d(-30, 0, Math.toRadians(0)));
            }
            if (gamepad1.right_trigger > 0.1) {
                if (speed > .5) speed -= .02;          //half speed
                if (speed < .5) speed += .02;

            } else if(gamepad1.left_trigger > 0.1){       //quarter speed
                if (speed > .28) speed -= .02;
                else if (speed < .28) speed += .02;

            } else {                                      //full speed
                if (speed < .7) speed += .04;
                if (speed < 1) speed -= .02;
                else if (speed > 1) speed += .02;
            }
            telemetry.addData("speed ", speed);
            // make drivetrain inputs non linear, not used
            double ProcessedTranslationY = -gamepad1.left_stick_y;
            double ProcessedTranslationX = -gamepad1.left_stick_x;


            // Movement calc Driver 1

            currentVector = new Vector2d(0, 0).rotated(-drive.getPoseEstimate().getHeading());

            if (gamepad1.dpad_up) {
                currentVector = new Vector2d(speed,
                        0).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_down) {
                currentVector = new Vector2d(-speed,
                        0).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_left) {
                currentVector = new Vector2d(0,
                        speed).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_right) {
                currentVector = new Vector2d(0,
                        -speed).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.right_trigger > .3) {
                currentVector = new Vector2d(-gamepad1.left_stick_y * speed,
                        -gamepad1.left_stick_x * speed).rotated(-drive.getPoseEstimate().getHeading());
            } else {
                currentVector = new Vector2d(-gamepad1.left_stick_y * speed,
                        -gamepad1.left_stick_x * speed).rotated(-drive.getPoseEstimate().getHeading());
            }
            gamepad1.rumble((int)(Math.abs(gamepad1.left_stick_x * (speed * 100) + Math.abs(gamepad1.left_stick_y * (speed * 100)))));



            //Heading calc Driver 1
            if (gamepad1.left_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() - Math.toRadians(2)));
            }
            else if (gamepad1.right_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() + Math.toRadians(2)));
            }

            // Driver 1 heading snap, fallback to stick control
            if (gamepad1.triangle) angle = 0;
            else if (gamepad1.square) angle = 90;
            else if (gamepad1.cross) angle = 180;
            else if (gamepad1.circle) angle = 270;
            else if (gamepad1.right_stick_x >.05 || gamepad1.right_stick_x < -.05 || gamepad1.right_stick_y >.05 || gamepad1.right_stick_y < -.05) {
                angle = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;
                timeFromLastInput.reset();
            } /*else if (timeFromLastInput.time() > 500) {
                    angle = Math.toDegrees(drive.getExternalHeading());}*/
            targetHeading = angle;
            pose = new Pose2d(currentVector, drive.HeadingCalculator_Angle(drive, angle));
            drive.setWeightedDrivePower(pose);
            drive.update();

        }
    }
}
