FROM ros:iron
LABEL authors="Ilia.Nechaev"

RUN apt update
RUN apt install -y python3 python3-pip python3-tk x11-xserver-utils

WORKDIR /app

COPY requirements.txt /app
RUN pip3 install -r requirements.txt

COPY src /app/src
COPY assets/ /app

RUN ./install.sh

