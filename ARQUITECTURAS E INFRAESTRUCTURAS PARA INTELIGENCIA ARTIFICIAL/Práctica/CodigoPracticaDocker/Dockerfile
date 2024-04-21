FROM --platform=linux/amd64 python:3.9.6-slim

WORKDIR /app

COPY ./requirements.txt /app/requirements.txt

RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get -y autoremove && \
    apt-get install -y \
        build-essential \
        libpq-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    pip install --no-cache-dir --upgrade -r /app/requirements.txt

COPY ./src /app/src
COPY ./api /app/api
COPY ./artifacts /app/artifacts

ENV DEFAULT_CONVERSION_RATE=270
ENV DEFAULT_SLOPE_CORRECTION=1.13

CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "7860"]
