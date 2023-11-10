/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

class CenterstageSuperPipeline implements VisionProcessor {
    protected static boolean leftSide;
    protected static boolean redAlliance;
    public static int spikeMark = 1;
    protected Point sub1PointA;
    protected Point sub1PointB;
    protected Point sub2PointA;
    protected Point sub2PointB;
    protected Point sub3PointA;
    protected Point sub3PointB;
    protected Mat YCrCb = new Mat();
    protected Mat colorChan = new Mat();
    protected Mat subMat1;
    protected Mat subMat2;
    protected Mat subMat3;
    protected int max;
    protected int avg1;
    protected int avg2;
    protected int avg3;
    protected Point spikeMarkCenter;

    public CenterstageSuperPipeline(boolean leftSide, boolean redAlliance)
    {
        this.leftSide = leftSide;
        this.redAlliance = redAlliance;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the image from RGB to YCrCb
        Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cr and Cb channels from the image
        if(redAlliance) {
            Core.extractChannel(YCrCb, colorChan, 0);
        } else {
            Core.extractChannel(YCrCb, colorChan, 2);
        }

        // The the sample areas fromt the Cb channel
        subMat1 = colorChan.submat(new Rect(sub1PointA, sub1PointB));
        subMat2 = colorChan.submat(new Rect(sub2PointA, sub2PointB));
        subMat3 = colorChan.submat(new Rect(sub3PointA, sub3PointB));

        // Average the sample areas
        avg1 = (int)Core.mean(subMat1).val[0];
        avg2 = (int)Core.mean(subMat2).val[0];
        avg3 = (int)Core.mean(subMat3).val[0];

        // Draw rectangles around the sample areas
        Imgproc.rectangle(frame, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(frame, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(frame, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);

        // Figure out which sample zone had the lowest contrast from color chan
        max = Math.max(avg1, Math.max(avg2, avg3));

        // Draw a circle on the detected skystone
        if(max == avg1) {
            spikeMarkCenter.x = (sub1PointA.x + sub1PointB.x) / 2;
            spikeMarkCenter.y = (sub1PointA.y + sub1PointB.y) / 2;
            Imgproc.circle(frame, spikeMarkCenter, 5, new Scalar(225, 52, 235), -1);
            spikeMark = 1;
        } else if(max == avg2) {
            spikeMarkCenter.x = (sub2PointA.x + sub2PointB.x) / 2;
            spikeMarkCenter.y = (sub2PointA.y + sub2PointB.y) / 2;
            Imgproc.circle(frame, spikeMarkCenter, 5, new Scalar(225, 52, 235), -1);
            spikeMark = 2;
        } else if(max == avg3) {
            spikeMarkCenter.x = (sub3PointA.x + sub3PointB.x) / 2;
            spikeMarkCenter.y = (sub3PointA.y + sub3PointB.y) / 2;
            Imgproc.circle(frame, spikeMarkCenter, 5, new Scalar(225, 52, 235), -1);
            spikeMark = 3;
        } else {
            spikeMark = 1;
        }

        // Free the allocated submat memory
        subMat1.release();
        subMat1 = null;
        subMat2.release();
        subMat2 = null;
        subMat3.release();
        subMat3 = null;

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}