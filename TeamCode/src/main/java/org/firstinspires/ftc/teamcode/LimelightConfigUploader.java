package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;

/**
 * Upload Limelight Configuration Files
 * <p>
 * This OpMode allows you to upload VPR (pipeline) and FMAP (field map) files
 * from the TeamCode/src/main/assets/limelight directory to specific pipeline slots on the Limelight.
 * <p>
 * Controls:
 * - D-Pad Up/Down: Navigate through options
 * - D-Pad Left/Right: Change values
 * - Triangle: Upload selected configuration
 * - Cross: Exit
 * <p>
 * Options:
 * 1. Pipeline Number (0-7): The slot where the configuration will be uploaded
 * 2. VPR File: Pipeline configuration file (.vpr)
 * 3. FMAP File: Field map file (.fmap) with AprilTag locations
 */
@TeleOp(name = "Limelight: Config Uploader", group = "Test")
public class LimelightConfigUploader extends LinearOpMode {

    private Limelight3A limelight;

    // Available files (discovered from res/limelight directory)
    private final List<String> vprFiles = new ArrayList<>();
    private final List<String> fmapFiles = new ArrayList<>();

    // User selections
    private int selectedPipeline = 0;
    private int selectedVprIndex = 0;
    private int selectedFmapIndex = 0;
    private int selectedOption = 0; // 0=pipeline, 1=vpr, 2=fmap

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // Discover available files
        discoverFiles();

        if (vprFiles.isEmpty()) {
            telemetry.addLine("ERROR: No VPR files found in res/limelight");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Limelight Config Uploader");
        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        boolean running = true;

        while (opModeIsActive() && running) {
            // Handle navigation
            if (gamepad1.dpadUpWasPressed()) {
                selectedOption = (selectedOption - 1 + 3) % 3;
            }
            if (gamepad1.dpadDownWasPressed()) {
                selectedOption = (selectedOption + 1) % 3;
            }

            // Handle value changes
            if (gamepad1.dpadLeftWasPressed()) {
                changeValue(-1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                changeValue(1);
            }

            // Handle upload
            if (gamepad1.triangleWasPressed()) {
                uploadConfiguration();
                sleep(500); // Debounce
            }

            // Handle exit
            if (gamepad1.crossWasPressed()) {
                running = false;
            }

            // Display UI
            updateTelemetry();
            sleep(50);
        }

        limelight.stop();
    }

    private void discoverFiles() {
        try {
            // List all files in the limelight assets directory
            String[] files = hardwareMap.appContext.getAssets().list("limelight");

            if (files != null) {
                for (String file : files) {
                    if (file.endsWith(".vpr")) {
                        vprFiles.add(file);
                    } else if (file.endsWith(".fmap")) {
                        fmapFiles.add(file);
                    }
                }
            }
        } catch (Exception e) {
            telemetry.addLine("Error discovering files: " + e.getMessage());
            telemetry.update();
        }

        // Always add "(none)" option for field maps
        fmapFiles.add("(none)");
    }

    private void changeValue(int delta) {
        switch (selectedOption) {
            case 0: // Pipeline number
                selectedPipeline = Math.max(0, Math.min(7, selectedPipeline + delta));
                break;
            case 1: // VPR file
                selectedVprIndex = (selectedVprIndex + delta + vprFiles.size()) % vprFiles.size();
                break;
            case 2: // FMAP file
                selectedFmapIndex = (selectedFmapIndex + delta + fmapFiles.size()) % fmapFiles.size();
                break;
        }
    }

    private void updateTelemetry() {
        telemetry.clear();
        telemetry.addLine("=== LIMELIGHT CONFIG UPLOADER ===");
        telemetry.addLine();

        // Pipeline number
        String pipelineMarker = (selectedOption == 0) ? ">>> " : "    ";
        telemetry.addLine(String.format("%sPipeline Number: %d", pipelineMarker, selectedPipeline));

        // VPR file
        String vprMarker = (selectedOption == 1) ? ">>> " : "    ";
        String vprName = vprFiles.isEmpty() ? "(none)" : vprFiles.get(selectedVprIndex);
        telemetry.addLine(String.format("%sVPR File: %s", vprMarker, vprName));

        // FMAP file
        String fmapMarker = (selectedOption == 2) ? ">>> " : "    ";
        String fmapName = fmapFiles.isEmpty() ? "(none)" : fmapFiles.get(selectedFmapIndex);
        telemetry.addLine(String.format("%sFMAP File: %s", fmapMarker, fmapName));

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  D-Pad Up/Down: Navigate");
        telemetry.addLine("  D-Pad Left/Right: Change value");
        telemetry.addLine("  Triangle: Upload config");
        telemetry.addLine("  Cross: Exit");
        telemetry.update();
    }

    private void uploadConfiguration() {
        telemetry.clear();
        telemetry.addLine("=== UPLOADING CONFIGURATION ===");
        telemetry.addLine();
        telemetry.addLine(String.format("Pipeline: %d", selectedPipeline));
        telemetry.addLine(String.format("VPR: %s", vprFiles.get(selectedVprIndex)));
        telemetry.addLine(String.format("FMAP: %s", fmapFiles.get(selectedFmapIndex)));
        telemetry.addLine();
        telemetry.update();

        boolean success = true;

        // Upload VPR file
        try {
            telemetry.addLine("Uploading VPR...");
            telemetry.update();

            String vprJson = loadResourceFile(vprFiles.get(selectedVprIndex));
            boolean vprResult = limelight.uploadPipeline(vprJson, selectedPipeline);

            if (vprResult) {
                telemetry.addLine("  VPR uploaded successfully");
            } else {
                telemetry.addLine("  VPR upload FAILED");
                success = false;
            }
            telemetry.update();
            sleep(500);
        } catch (Exception e) {
            telemetry.addLine("  ERROR: " + e.getMessage());
            telemetry.update();
            success = false;
        }

        // Upload FMAP file if not "(none)"
        String selectedFmap = fmapFiles.get(selectedFmapIndex);
        if (!selectedFmap.equals("(none)")) {
            try {
                telemetry.addLine("Uploading FMAP...");
                telemetry.update();

                String fmapJson = loadResourceFile(selectedFmap);
                JSONObject fmapObject = new JSONObject(fmapJson);

                // Use reflection to call protected constructor
                Constructor<LLFieldMap> constructor = LLFieldMap.class.getDeclaredConstructor(JSONObject.class);
                constructor.setAccessible(true);
                LLFieldMap fieldMap = constructor.newInstance(fmapObject);

                boolean fmapResult = limelight.uploadFieldmap(fieldMap, selectedPipeline);

                if (fmapResult) {
                    telemetry.addLine("  FMAP uploaded successfully");
                } else {
                    telemetry.addLine("  FMAP upload FAILED");
                    success = false;
                }
                telemetry.update();
                sleep(500);
            } catch (Exception e) {
                telemetry.addLine("  ERROR: " + e.getMessage());
                telemetry.update();
                success = false;
            }
        }

        // Final result
        telemetry.addLine();
        if (success) {
            telemetry.addLine("=== UPLOAD COMPLETE ===");
            telemetry.addLine("Configuration uploaded successfully!");
            telemetry.addLine(String.format("Switch to pipeline %d to use it.", selectedPipeline));
        } else {
            telemetry.addLine("=== UPLOAD FAILED ===");
            telemetry.addLine("Check errors above.");
        }
        telemetry.addLine();
        telemetry.addLine("Press any button to continue...");
        telemetry.update();

        // Wait for button press
        while (opModeIsActive() && !gamepad1.atRest()) {
            sleep(50);
        }
        while (opModeIsActive() && gamepad1.atRest()) {
            sleep(50);
        }
    }

    /**
     * Load a file from assets (TeamCode/src/main/assets/limelight)
     */
    private String loadResourceFile(String filename) throws Exception {
        try (InputStream inputStream = hardwareMap.appContext.getAssets().open("limelight/" + filename)) {
            return readInputStream(inputStream);
        } catch (Exception e) {
            throw new Exception("Failed to load " + filename + ": " + e.getMessage());
        }
    }

    /**
     * Read an input stream into a string
     */
    private String readInputStream(InputStream inputStream) throws Exception {
        BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
        StringBuilder content = new StringBuilder();
        String line;

        while ((line = reader.readLine()) != null) {
            content.append(line).append("\n");
        }

        reader.close();
        inputStream.close();
        return content.toString();
    }
}
