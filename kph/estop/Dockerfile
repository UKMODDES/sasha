FROM sasha-core:latest

# Copy python scripts
COPY requirements.txt /app/estop_requirements.txt
RUN python3 -m pip install -r /app/estop_requirements.txt
COPY *.py /app/

ENTRYPOINT ["/usr/bin/python3", "/app/estop_nogui.py", "192.168.80.3"]
