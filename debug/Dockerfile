FROM cmu-mars/base
RUN sudo apt-get update
RUN sudo apt-get install -y python3 python3-pip

RUN sudo mkdir -p /usr/src/app
RUN sudo chown mars /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN sudo pip3 install --no-cache-dir -r requirements.txt

ADD main.py /usr/src/app/main.py

RUN sudo chown -R mars /usr/src/app

ENV PYTHONUNBUFFERED 0

CMD ["python3", "main.py"]
