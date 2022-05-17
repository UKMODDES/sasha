FROM sasha-core:latest

# Copy python scripts
COPY spot_common /app/spot_common
RUN python3 -m pip install /app/spot_common

ENTRYPOINT ["/usr/bin/python3", "-m", "spot_common/spot_functions/take_picture_spot_function.py"]
