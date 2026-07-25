# HTTP -> MQTT/HA bridge for the GPS tracker heartbeat
FROM python:3.12-slim
WORKDIR /app
RUN pip install --no-cache-dir "paho-mqtt==1.6.1"   # v1 callback API (matches ha_bridge.py)
COPY ha_bridge.py .
CMD ["python", "-u", "ha_bridge.py"]
