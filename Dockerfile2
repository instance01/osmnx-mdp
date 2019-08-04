FROM python:3.7.3-slim-stretch

WORKDIR /app

RUN pip install pytest osmnx networkx numpy cython
RUN apt-get update && apt-get install -y libspatialindex-c4v5 g++ libsparsehash-dev vim gdb tk

COPY contrib/bashrc /root/.bashrc
COPY contrib/vimrc /root/.vimrc
COPY contrib/matplotlibrc /root/matplotlibrc
