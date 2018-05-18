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

RUN pip install --no-cache numpy==1.14.2

## install BugZoo from source
#ENV BUGZOO_REVISION 
#RUN git clone https://github.com/squaresLab/BugZoo /tmp/bugzoo --depth 30 \
# && cd /tmp/bugzoo \
# && git fetch origin "${BUGZOO_REVISION}" \
# && git checkout "${BUGZOO_REVISION}" \
# && pip install --no-cache . \
# && rm -rf /tmp/bugzoo
#
## install boggart from source
#ENV BOGGART_REVISION a8820c79cdd21deb4200db74b4c1fd742f728a4b
#RUN git clone https://github.com/squaresLab/boggart /tmp/boggart \
# && cd /tmp/boggart \
# && git fetch origin "${BOGGART_REVISION}" \
# && git checkout "${BOGGART_REVISION}" \
# && pip install --no-cache . \
# && rm -rf /tmp/boggart
#
## install Darjeeling from source
#ENV DARJEELING_REVISION 731423982bc030aaeaa4fa559e76be69cf7c31b3
#RUN git clone https://github.com/squaresLab/darjeeling /tmp/darjeeling \
# && cd /tmp/darjeeling \
# && git fetch origin "${DARJEELING_REVISION}" \
# && git checkout "${DARJEELING_REVISION}" \
# && pip install --no-cache . \
# && rm -rf /tmp/darjeeling

# install Orchestrator from source
ENV ORCHESTRATOR_REVISION rr2-iw
RUN git clone https://github.com/cmu-mars/p2c2-orchestrator /tmp/orchestrator \
 && cd /tmp/orchestrator \
 && git fetch origin "${ORCHESTRATOR_REVISION}" \
 && git checkout "${ORCHESTRATOR_REVISION}" \
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
