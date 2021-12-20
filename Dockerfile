# The base image
FROM python:3 as builder
WORKDIR /workspace
RUN apt update && apt install ffmpeg libsm6 libxext6  -y

RUN pip install opencv-python numpy opencv_contrib_python


FROM builder AS runtime
#Installing the application
ADD src src

# Define entrypoint
ENTRYPOINT ["/usr/bin/python3","src/main.py"]
