FROM sasha-core:latest

# Copy python scripts
COPY *.py /app/
COPY person.jpeg /app/
ENV TF_CPP_MIN_VLOG_LEVEL=2

ENTRYPOINT ["/usr/bin/python3", "/app/tensorflow_detect.py"]
