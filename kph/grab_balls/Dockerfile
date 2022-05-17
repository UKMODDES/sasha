FROM sasha-core:latest

# Copy python scripts
COPY spot_common /app/spot_common
RUN python3 -m pip install /app/spot_common

COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install -r /app/requirements.txt

ENV PYTHONPATH=/app/

COPY ball_detect.py /app/

ENTRYPOINT ["/usr/bin/python3", "/app/ball_detect.py"]
