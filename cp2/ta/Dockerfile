FROM cmu-mars/orchestrator

RUN sudo apt-get install -y python3 python3-pip

RUN sudo mkdir -p /usr/src/app
RUN sudo chown mars /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN sudo pip3 install --no-cache-dir -r requirements.txt

COPY . /usr/src/app

RUN sudo chown -R mars /usr/src/app

ENV PYTHONUNBUFFERED 0

EXPOSE 8080

CMD ["python3", "-m", "swagger_server"]
