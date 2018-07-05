FROM alpine:3.7

# we add a custom entrypoint script to allow us to access the host
COPY entrypoint.sh /usr/bin/
ENTRYPOINT ["/usr/bin/entrypoint.sh"]

# credit to: https://hub.docker.com/r/frolvlad/alpine-python3
RUN apk add --no-cache \
      python3 \
      python3-dev \
      git \
      gcc \
      linux-headers \
      musl-dev \
 && python3 -m ensurepip \
 && rm -r /usr/lib/python*/ensurepip \
 && pip3 install --no-cache --upgrade pip==9.0.3 setuptools \
 && if [[ ! -e /usr/bin/pip ]]; then ln -s pip3 /usr/bin/pip ; fi \
 && if [[ ! -e /usr/bin/python ]]; then ln -sf /usr/bin/python3 /usr/bin/python; fi

# install BugZoo from source
COPY --from=squareslab/bugzoo /opt/bugzoo /tmp/bugzoo
RUN cd /tmp/bugzoo \
 && pip install --no-cache . \
 && rm -rf /tmp/bugzoo

# install boggart from source
COPY --from=squareslab/boggart /opt/boggart /tmp/boggart
RUN cd /tmp/boggart \
 && pip install --no-cache . \
 && rm -rf /tmp/boggart

# install Kaskara from source
COPY --from=squareslab/kaskara /opt/kaskara /tmp/kaskara
RUN cd /tmp/kaskara \
 && pip install --no-cache . \
 && rm -rf /tmp/kaskara

# install Darjeeling from source
COPY --from=squareslab/darjeeling /opt/darjeeling /tmp/darjeeling
RUN cd /tmp/darjeeling \
 && pip install --no-cache . \
 && rm -rf /tmp/darjeeling

# install Orchestrator from source
COPY --from=cmumars/orchestrator /opt/orchestrator /tmp/orchestrator
RUN cd /tmp/orchestrator \
 && pip install --no-cache . \
 && rm -rf /tmp/orchestrator

# install the TA
RUN mkdir -p /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /usr/src/app

ENV PYTHONUNBUFFERED 0

CMD ["python3", "-m", "swagger_server"]
