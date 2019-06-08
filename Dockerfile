FROM python:3.7.3-slim-stretch

WORKDIR /app

COPY . /app

RUN pip install --trusted-host pypi.python.org -r requirements.txt
RUN pip install pytest
RUN apt-get update && apt-get install -y libspatialindex-c4v5 g++ libsparsehash-dev
RUN cd /app && pip install -e .
