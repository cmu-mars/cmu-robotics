FROM alpine:3.7

# we add a custom entrypoint script to allow us to access the host
COPY entrypoint.sh /usr/bin/
ENTRYPOINT ["/usr/bin/entrypoint.sh"]

# credit to: https://hub.docker.com/r/frolvlad/alpine-python3
RUN apk add --no-cache python3 git gcc gfortran python3-dev build-base openblas-dev \
 && python3 -m ensurepip \
 && rm -r /usr/lib/python*/ensurepip \
 && pip3 install --no-cache --upgrade pip==9.0.3 setuptools \
 && if [[ ! -e /usr/bin/pip ]]; then ln -s pip3 /usr/bin/pip ; fi \
 && if [[ ! -e /usr/bin/python ]]; then ln -sf /usr/bin/python3 /usr/bin/python; fi

RUN pip install --no-cache numpy

# install Hulk from source
ENV HULK_REVISION rr2
RUN git clone https://github.com/squaresLab/Hulk /tmp/hulk \
 && cd /tmp/hulk \
 && git checkout "${HULK_REVISION}" \
 && pip install --no-cache . \
 && rm -rf /tmp/hulk

# install Orchestrator from source
ENV ORCHESTRATOR_REVISION rr2
RUN git clone https://github.com/cmu-mars/p2c2-orchestrator /tmp/orchestrator \
 && cd /tmp/orchestrator \
 && git checkout "${ORCHESTRATOR_REVISION}" \
 && pip install --no-cache --upgrade . \
 && rm -rf /tmp/orchestrator

# install the TA
RUN mkdir -p /usr/src/app
WORKDIR /usr/src/app

COPY requirements.txt /usr/src/app/

RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /usr/src/app

ENV PYTHONUNBUFFERED 0

CMD ["python3", "-m", "swagger_server"]
