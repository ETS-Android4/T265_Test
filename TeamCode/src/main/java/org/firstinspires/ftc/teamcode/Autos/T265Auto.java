package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Drive.Robot;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous()
public class T265Auto extends LinearOpMode {
    public Robot robot = null;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, "autonomous");

        waitForStart();

        robot.setPoseEstimate(new Pose2d(0.0, 0.0, Math.toRadians(0)));

        TrajectorySequence trajectorySequence0 = robot.trajectorySequenceBuilder(new Pose2d(0.0, 0.0, Math.toRadians(0)))
                .splineTo(new Vector2d(24,24), 0)
                .turn(Math.toRadians(90))
                .build();

        robot.followTrajectorySequence(trajectorySequence0);
        telemetry.addData("what i think ", robot.getPoseEstimate());
        telemetry.addData("End ", robot.getPoseEstimate().minus(new Pose2d(24,24, Math.toRadians(90))));
        telemetry.update();
        sleep(20 * 1000);
    }
}
