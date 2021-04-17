package frc.team88.swerve.util.logging;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Map.Entry;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Class for logging data to a JSONL file on the RIO.
 */
public class DataLogger {

    private static DataLogger instance;

    private static final Path LOGS_DIR = Paths.get("/data/");
    private static final Path LAST_LOG_PATH = LOGS_DIR.resolve("last_log_num.txt");

    private Path logPath;

    private Gson gson;

    private Map<String, Object> dataToLog;

    /**
     * Initializes the DataLogger instance.
     */
    public static void initialize() {
        instance = new DataLogger();
    }

    /**
     * Gets the DataLogger singleton instance.
     */
    public static DataLogger getInstance() {
        if (Objects.isNull(instance)) {
            initialize();
        }
        return instance;
    }

    /**
     * Adds data to be included in the next log line.
     * 
     * @param label
     *                  The string to be used as a label for this data.
     * 
     * @param data
     *                  Any Java object which can be encoded into JSON.
     */
    public void addData(String label, Object data) {
        this.dataToLog.put(label, data);
    }

    /**
     * Writes all added data since the last call to a new line in the log file.
     */
    public void logData() {
        if (Objects.isNull(logPath)) {
            System.err.println("Cannot write to log because the file failed to initialize.");
            this.dataToLog.clear();
            return;
        }

        // Create POJO for data representation.
        class LogItem {
            String label;
            Object data;

            public LogItem(String label, Object data) {
                this.label = label;
                this.data = data;
            }
        }

        // Start with a timestamp object.
        List<LogItem> logItems = new LinkedList<>();
        logItems.add(new LogItem("Timestamp", RobotController.getFPGATime()));

        // Add the rest of the items
        for (Entry<String, Object> entry : this.dataToLog.entrySet()) {
            logItems.add(new LogItem(entry.getKey(), entry.getValue()));
        }

        // Write line to log file.
        try {
            Files.writeString(this.logPath, this.gson.toJson(logItems) + "/n");
        } catch (IOException err) {
            System.err.format("Encountered IO Exception when writing logs: %s%n", err);
        }

        // Clear old data.
        this.dataToLog.clear();
    }

    /**
     * Initializes the data logger, opening a file to log data to.
     */
    private DataLogger() {
        this.gson = new GsonBuilder().setPrettyPrinting().create();
        this.dataToLog = new HashMap<>();

        // Ensure the data directory exists
        try {
            if (Files.notExists(LOGS_DIR)) {
                Files.createDirectories(LOGS_DIR);
            }
        } catch (IOException err) {
            System.err.format("Encountered IO Exception when ensuring existence of log directory %s: %s%n",
                    LOGS_DIR.toString(), err);
            this.logPath = null;
            return;
        }

        // Get and update the current log number from file
        int logNum;
        try {
            if (Files.exists(LAST_LOG_PATH)) {
                logNum = Integer.parseInt(Files.readAllLines(LAST_LOG_PATH).get(0));
                logNum++;
                Files.writeString(LAST_LOG_PATH, Integer.toString(logNum));
            } else {
                logNum = 0;
                Files.writeString(LAST_LOG_PATH, Integer.toString(logNum), StandardOpenOption.CREATE_NEW);
            }
        } catch (IOException err) {
            System.err.format("Encountered IO Exception when getting/updating log number at %s: %s%n",
                    LAST_LOG_PATH.toString(), err);
            this.logPath = null;
            return;
        }

        // Create the log file
        this.logPath = LOGS_DIR.resolve(Integer.toString(logNum) + ".jsonl");
        try {
            Files.createFile(this.logPath);
        } catch (IOException err) {
            System.err.format("Encountered IO Exception when creating the log file %s: %s%n", LAST_LOG_PATH.toString(),
                    err);
            this.logPath = null;
            return;
        }
    }

}