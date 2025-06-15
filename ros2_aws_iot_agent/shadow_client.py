import sys
import asyncio
import logging
import threading
import traceback
import random
from uuid import uuid4
from dataclasses import dataclass
from awscrt import mqtt
from awsiot import iotshadow, mqtt_connection_builder
import json


# Configure logger
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# Using globals to simplify sample code
is_sample_done = threading.Event()

SHADOW_VALUE_DEFAULT = {}


@dataclass
class ShadowClientConfig:
    thing_name: str
    endpoint: str
    cert: str
    key: str
    ca: str


class ShadowClient:
    def __init__(self, config: ShadowClientConfig, default_shadow: dict):
        self.config = config
        self.default_shadow = default_shadow
        self.lock = threading.Lock()
        self.shadow_state = dict(default_shadow)
        self.request_tokens = set()
        self.connected = False

        self._init_mqtt()

    def _init_mqtt(self):
        self.mqtt_connection = mqtt_connection_builder.mtls_from_path(
            endpoint=self.config.endpoint,
            cert_filepath=self.config.cert,
            pri_key_filepath=self.config.key,
            ca_filepath=self.config.ca,
            client_id=self.config.thing_name,
            clean_session=False,
            keep_alive_secs=30
        )
        self.shadow_client = iotshadow.IotShadowClient(self.mqtt_connection)

    def _safe_callback(self, func):
        """Wrapper to safely run callbacks and catch exceptions."""
        def wrapper(event):
            try:
                func(event)
            except Exception as e:
                logger.error(f"Exception in callback {func.__name__}: {e}")
                logger.error(traceback.format_exc())
        return wrapper

    async def _connect_and_run(self):
        """Connect to AWS IoT Core and start listening to shadow topics."""
        logger.info("Connecting to AWS IoT Core...")
        await asyncio.to_thread(self.mqtt_connection.connect)
        self.connected = True
        logger.info("Successfully connected.")

        await self._subscribe_all()
        await self._get_initial_shadow()

        while self.connected:
            await asyncio.sleep(1)

    async def _subscribe_all(self):
        """Subscribe to all necessary shadow topics."""
        logger.info("Subscribing to shadow topics...")
        try:
            await asyncio.to_thread(self.shadow_client.subscribe_to_get_shadow_accepted,
                iotshadow.GetShadowSubscriptionRequest(self.config.thing_name),
                mqtt.QoS.AT_LEAST_ONCE, self._safe_callback(self._on_get_accepted))

            await asyncio.to_thread(self.shadow_client.subscribe_to_get_shadow_rejected,
                iotshadow.GetShadowSubscriptionRequest(self.config.thing_name),
                mqtt.QoS.AT_LEAST_ONCE, self._safe_callback(self._on_get_rejected))

            await asyncio.to_thread(self.shadow_client.subscribe_to_update_shadow_accepted,
                iotshadow.UpdateShadowSubscriptionRequest(self.config.thing_name),
                mqtt.QoS.AT_LEAST_ONCE, self._safe_callback(self._on_update_accepted))

            await asyncio.to_thread(self.shadow_client.subscribe_to_update_shadow_rejected,
                iotshadow.UpdateShadowSubscriptionRequest(self.config.thing_name),
                mqtt.QoS.AT_LEAST_ONCE, self._safe_callback(self._on_update_rejected))

            await asyncio.to_thread(self.shadow_client.subscribe_to_shadow_delta_updated_events,
                iotshadow.ShadowDeltaUpdatedSubscriptionRequest(self.config.thing_name),
                mqtt.QoS.AT_LEAST_ONCE, self._safe_callback(self._on_delta))

            logger.info("All subscriptions successfully established.")
        except Exception as e:
            logger.error("Failed to subscribe to shadow topics.")
            raise e

    async def _get_initial_shadow(self):
        """Request the initial shadow state."""
        try:
            token = str(uuid4())
            self.request_tokens.add(token)
            request = iotshadow.GetShadowRequest(
                thing_name=self.config.thing_name, client_token=token
            )
            await asyncio.to_thread(
                self.shadow_client.publish_get_shadow, request, mqtt.QoS.AT_LEAST_ONCE
            )
            logger.info("Requested initial shadow state.")
        except Exception as e:
            logger.error(f"Failed to get initial shadow: {e}")
            raise e

    def _apply_shadow_delta(self, delta: dict):
        """Apply delta received from AWS IoT shadow."""
        logger.info(f"Delta received: {delta}")
        with self.lock:
            for key, value in delta.items():
                self.shadow_state[key] = value
        self._sync_reported_state()

    def _sync_reported_state(self):
        """Publish updated reported state to AWS IoT shadow."""
        try:
            state = iotshadow.ShadowState(
                reported=self.shadow_state,
                desired=self.shadow_state
            )
            token = str(uuid4())
            self.request_tokens.add(token)
            request = iotshadow.UpdateShadowRequest(
                thing_name=self.config.thing_name,
                state=state,
                client_token=token
            )
            self.shadow_client.publish_update_shadow(request, mqtt.QoS.AT_LEAST_ONCE)
            logger.info("Reported state successfully updated.")
        except Exception as e:
            logger.error(f"Failed to publish reported state: {e}")
            logger.error(traceback.format_exc())

    # --- 실제 콜백들 ---
    def _on_get_accepted(self, response):
        logger.info("Received initial shadow state.")
        merged = dict(self.default_shadow)
        if response.state:
            if response.state.reported:
                merged.update(response.state.reported)
            if response.state.delta:
                merged.update(response.state.delta)
        with self.lock:
            self.shadow_state = merged
        self._sync_reported_state()

        self.log_shadow_state()

    def _on_get_rejected(self, error):
        logger.warning(f"Get rejected: {error.code} - {error.message}")
        self.log_shadow_state()

    def _on_update_accepted(self, response):
        logger.info("Update accepted.")
        self.log_shadow_state()

    def _on_update_rejected(self, error):
        logger.warning(f"Update rejected: {error.code} - {error.message}")
        self.log_shadow_state()

    def _on_delta(self, delta_event):
        if delta_event.state:
            self._apply_shadow_delta(delta_event.state)

        self.log_shadow_state()

    async def start(self):
        backoff = 1

        while True:
            try:
                await self._connect_and_run()
                backoff = 1  # 성공시 backoff 초기화
            except Exception as e:
                logger.error(f"Connection error: {e}")
                logger.error(traceback.format_exc())
                logger.info(f"Reconnecting after {backoff} seconds...")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 60) + random.uniform(0, 1)

    async def stop(self):
        """Stop client and disconnect."""
        self.connected = False
        await asyncio.to_thread(self.mqtt_connection.disconnect)
        logger.info("Disconnected from AWS IoT Core.")

    def update_device_state(self, new_state: dict):
        """Update device state and report to AWS IoT Shadow."""

        try:
            logger.info(f"Updating shadow with new state: {new_state}")

            # Update the reported state with the new values
            with self.lock:
                self.shadow_state.update(new_state)  # reported 상태를 변경

            # Reflect the reported state back to the cloud
            state = iotshadow.ShadowState(
                reported=self.shadow_state
            )

            token = str(uuid4())
            self.request_tokens.add(token)

            # Send the update request to the shadow
            request = iotshadow.UpdateShadowRequest(
                thing_name=self.config.thing_name,
                state=state,
                client_token=token
            )
            self.shadow_client.publish_update_shadow(request, mqtt.QoS.AT_LEAST_ONCE)
            logger.info("UpdateShadow request successfully published.")

        except Exception as e:
            logger.error(f"Failed to update shadow state: {e}")
            logger.error(traceback.format_exc())

    def log_shadow_state(self):
        """
        Log the current internal shadow state.
        """
        with self.lock:
            state_json = json.dumps(self.shadow_state, indent=2)
            logger.info("Current internal shadow state:\n%s", state_json)
