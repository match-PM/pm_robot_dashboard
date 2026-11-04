# pm_robot_dashboard

RSAP log analysis is available under **Info Helpers → RSAP Log Analysis**.
Import or drag and drop RSAP JSON logs in the Log files tab, or connect to an
`rsap_interfaces/msg/RsapLogs` topic in the Live topic tab. Select individual
actions or entire runs, exclude failures, and compare duration charts and statistics.
The analysis and widget are provided by `pm_robot_dashboard.log_analysis` and
`pm_robot_dashboard.log_analysis_widget`. Rebuild `pm_robot_dashboard` and
`rsap_interfaces` after updating. The widget requires PyQt6 and Matplotlib.

Live messages and exported files contain execution history, including repeated
actions with distinct `execution_id` values. Exporting preserves the history;
starting a sequence run or clearing logs resets it. Overlapping file snapshots
are deduplicated by execution ID, with each execution attributed to the first
loaded file containing it. Legacy logs without IDs remain independent.
