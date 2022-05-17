FROM sasha-core:latest

# Copy python scripts
COPY *.py /app/

ENTRYPOINT ["/usr/bin/python3", "/app/spot_detect_and_follow.py", "--model-path", "/app/models/ssd.pb", "--detection-class", "1", "--sleep-between-capture", "1.0", "$ROBOT_IP"]
