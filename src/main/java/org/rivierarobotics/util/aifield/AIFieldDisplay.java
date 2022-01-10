/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util.aifield;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Adds the "AI Mesh" Camera to the CameraServer which is
 * able to be updated with paths as the robot calculates them.
 * The Thread is only here for testing purposes only,
 * Updating the path should be done by calling updatePath
 */
public class AIFieldDisplay implements Runnable {
    private final CvSource outputStream;
    private final FieldMesh fieldMesh;
    private final int imgWidth;
    private final int imgHeight;
    private final double scalingRatio;
    private final int updateRate;
    private static final List<FieldNode> path = Collections.synchronizedList(new ArrayList<>());
    private static volatile Trajectory generatedTrajectory;
    private static volatile boolean needsToRender = false;
    private static ScheduledExecutorService mainImageThread;

    public AIFieldDisplay(FieldMesh fieldMesh, int updateRate) throws IOException {
        int scalingFactor = 1000;
        this.updateRate = updateRate;
        imgHeight = scalingFactor;
        this.fieldMesh = fieldMesh;
        imgWidth = (int) (scalingFactor * (((double) fieldMesh.fieldWidth) / fieldMesh.fieldHeight));
        scalingRatio = (double) scalingFactor / fieldMesh.fieldHeight;
        this.outputStream = CameraServer.putVideo("AI Mesh", imgWidth, imgHeight);
        mainImageThread = Executors.newSingleThreadScheduledExecutor();
        startFieldThread(updateRate);
    }

    public void refresh() throws InterruptedException {
        mainImageThread.shutdown();
        mainImageThread.awaitTermination(10, TimeUnit.SECONDS);
        mainImageThread = Executors.newSingleThreadScheduledExecutor();
        startFieldThread(updateRate);
    }

    private void startFieldThread(int updateRate) {
        Mat fieldMat = new Mat(imgHeight, imgWidth, CvType.CV_8UC(4), Scalar.all(100));
        Mat fieldRef = createField();
        Mat resized = new Mat();
        fieldRef.copyTo(fieldMat);
        mainImageThread.scheduleAtFixedRate(
                () -> {
                    if (needsToRender) {
                        fieldRef.copyTo(fieldMat);
                        List<FieldNode> tmp = path;
                        if (generatedTrajectory != null) {
                            for (double i = 0; i < generatedTrajectory.getTotalTimeSeconds(); i += 0.1) {
                                if (generatedTrajectory.getTotalTimeSeconds() < i + 0.1) {
                                    continue;
                                }
                                var pose1 = generatedTrajectory.sample(i);
                                var pose2 = generatedTrajectory.sample(i + 0.1);

                                Imgproc.arrowedLine(
                                        fieldMat,
                                        new Point(pose1.poseMeters.getX() * 100 * scalingRatio, pose1.poseMeters.getY() * 100 * scalingRatio),
                                        new Point(pose2.poseMeters.getX() * 100 * scalingRatio, pose2.poseMeters.getY() * 100 * scalingRatio),
                                        new Scalar(0, 0, 255 * (generatedTrajectory.sample(i).velocityMetersPerSecond) / 2), 10);
                            }
                        }

                        needsToRender = false;
                    }
                    outputStream.putFrame(fieldMat);
                }, 0, updateRate, TimeUnit.MILLISECONDS
        );
    }

    public void updatePath(Trajectory trajectory) {
        //AIFieldDisplay.path.clear();
        //AIFieldDisplay.path.addAll(path);
        generatedTrajectory = trajectory;
        needsToRender = true;
    }

    private Mat createField() {
        Mat field = new Mat(imgHeight, imgWidth, CvType.CV_8UC(4), Scalar.all(100));

        drawFieldMesh(field);
        drawFieldObstacles(field);
        drawWeightedAreas(field);

        return field;
    }

    private void drawFieldObstacles(Mat field) {
        Scalar color = new Scalar(64, 64, 64);
        int lineType = Imgproc.LINE_8;
        var fieldObstacles = fieldMesh.getObstacles();
        for (var obstacle : fieldObstacles) {
            List<MatOfPoint> list = new ArrayList<>();
            Point[] pts = new Point[obstacle.npoints];
            for (int i = 0; i < obstacle.npoints; i++) {
                pts[i] = (new Point(obstacle.xpoints[i] * scalingRatio, obstacle.ypoints[i] * scalingRatio));
            }
            list.add(new MatOfPoint(pts));
            Imgproc.fillPoly(field, list, color, lineType);
        }
    }

    private void drawWeightedAreas(Mat field) {
        var weightedAreas = fieldMesh.getAreaWeights();
        for (var aw : weightedAreas) {
            Imgproc.rectangle(
                    field,
                    new Point(aw.x1 * scalingRatio, aw.y1 * scalingRatio),
                    new Point(aw.x2 * scalingRatio, aw.y2 * scalingRatio),
                    new Scalar(0, 0, 255 * (aw.weight / 20.0)),
                    3);
        }
    }

    private void drawFieldMesh(Mat field) {
        var nodes = fieldMesh.getFieldNodes();
        for (var n : nodes) {
            for (var p : n) {
                if (!p.isValid) {
                    continue;
                }
                for (var e : p.neighbors) {
                    Point a = new Point(p.xValue * scalingRatio, p.yValue * scalingRatio);
                    Point b = new Point(e.node.xValue * scalingRatio, e.node.yValue * scalingRatio);
                    if (p.nodeWeight < 0) {
                        Imgproc.line(field, a, b, new Scalar(255, 0, 0, 1));
                    } else {
                        Imgproc.line(field, a, b, new Scalar(0, 255 - (255 * (p.nodeWeight) / 60.0), 255 * (p.nodeWeight) / 60.0), 1);
                    }
                }
            }
        }
    }

    private static boolean obstacles = false;
    private static ScheduledExecutorService exe = Executors.newSingleThreadScheduledExecutor();

    @Override
    public void run() {
        ScheduledExecutorService exe1 = Executors.newSingleThreadScheduledExecutor();
        exe1.scheduleAtFixedRate(
                () -> {
                    if (!exe.isShutdown()) {
                        exe.shutdown();
                    }
                    try {
                        exe.awaitTermination(1, TimeUnit.SECONDS);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (obstacles) {
                        for (int i = 0; i < 4; i++) {
                            fieldMesh.removeObstacle(fieldMesh.getObstacles().get(fieldMesh.getObstacles().size() - 1));
                        }
                    }
                    for (int i = 0; i < 4; i++) {
                        int xpos = (int) (fieldMesh.fieldWidth * Math.random());
                        int ypos = (int) (fieldMesh.fieldHeight * Math.random());

                        var x1 = new int[]{xpos - 75, ypos - 75};
                        var x2 = new int[]{xpos + 75, ypos - 75};
                        var x3 = new int[]{xpos + 75, ypos + 75};
                        var x4 = new int[]{xpos - 75, ypos + 75};
                        ArrayList<int[]> list = new ArrayList<>();
                        list.add(x1);
                        list.add(x2);
                        list.add(x3);
                        list.add(x4);
                        fieldMesh.addObstacle(list);
                    }
                    obstacles = true;
                    try {
                        refresh();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    exe = Executors.newSingleThreadScheduledExecutor();
                    exe.scheduleWithFixedDelay(
                            () -> {
                                int count = 0;
                                //System.out.println("Started");
                                var time = Timer.getFPGATimestamp();
                                Trajectory gen = null;
                                while (gen == null) {
                                    count++;
                                    if (count > 5000) {
                                        break;
                                    }
                                    int startx = (int) (0);
                                    int endx = (int) (fieldMesh.fieldWidth * Math.random());
                                    int starty = (int) (0);
                                    int endy = (int) (fieldMesh.fieldHeight * Math.random());
                                    //tmp = fieldMesh.getPath(startx, starty, endx, endy);
                                    gen = fieldMesh.getTrajectory(startx / 100.0, starty / 100.0, endx / 100.0, endy / 100.0, true, 0);
                                }
                                //System.out.println("Path Generation took: " + new DecimalFormat("0.00").format((Timer.getFPGATimestamp() - time) * 100) + "ms");
                                if (gen != null) {
                                    updatePath(gen);
                                }
                            }, 0, 200, TimeUnit.MILLISECONDS
                    );
                }, 0, 5, TimeUnit.SECONDS
        );
    }
}
