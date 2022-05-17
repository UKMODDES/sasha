FROM sasha-core:latest

# Copy python scripts
COPY spot_common /app/spot_common
RUN python3 -m pip install /app/spot_common

COPY requirements.txt /app/requirements.txt
RUN python3 -m pip install -r /app/requirements.txt

ENV PYTHONPATH=/app/

ENTRYPOINT ["/usr/bin/python3", "-m", "spot_common.spot_functions.draw_circles_spot_function", "192.168.80.3"]
