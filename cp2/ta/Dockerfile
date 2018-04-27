FROM cmu-mars/orchestrator

# we add a custom entrypoint script to allow us to access the host
COPY entrypoint.sh /usr/bin/
ENTRYPOINT ["/usr/bin/entrypoint.sh"]

RUN mkdir -p /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /usr/src/app

ENV PYTHONUNBUFFERED 0

CMD ["python3", "-m", "swagger_server"]
