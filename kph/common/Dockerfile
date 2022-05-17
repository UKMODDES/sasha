# Start at Tensorflow Image
FROM tensorflow/tensorflow:2.1.1-gpu

# Update Nvidia GPG Keys and update apt-get packages
RUN apt-get install -y wget && apt-get install -y dpkg && apt-key del 7fa2af80
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb && dpkg -i cuda-keyring_1.0-1_all.deb
RUN apt-get update && apt-get install -y libsm6 libxext6 libxrender-dev libgl1-mesa-glx

# Pull in and pip install requirements.txt
COPY requirements.txt /app/
RUN python3 -m pip install -r /app/requirements.txt

# Grab Both Models
COPY models/*.pb /app/models/

# Setup Environment Variables
ENV LANG en_US.UTF-8
ENV BOSDYN_CLIENT_USERNAME=Sasha
ENV BOSDYN_CLIENT_PASSWORD=olympicsasha1
ENV ROBOT_IP="192.168.80.3"

ENTRYPOINT []

