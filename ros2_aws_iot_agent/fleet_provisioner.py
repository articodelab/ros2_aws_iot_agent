import time
import logging
from typing import Optional
from dataclasses import dataclass
from awscrt import mqtt, http
from awsiot import iotidentity, mqtt_connection_builder

# Configure logger
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class FleetProvisioningResult:
    thing_name: str
    certificate_id: str
    certificate_pem: str
    private_key: Optional[str]


@dataclass
class FleetProvisioningConfig:
    thing_name: str
    endpoint: str
    port: int
    proxy_host: Optional[str]
    proxy_port: Optional[int]
    claim_cert: str
    claim_key: str
    ca: str
    csr: Optional[str]
    template_name: Optional[str]
    template_parameters: Optional[dict]
    mqtt_version: int


class FleetProvisioner:
    def __init__(self, config: FleetProvisioningConfig):
        self.config = config
        self.identity_client = None
        self.mqtt_connection = self._build_mqtt_connection()
        self.createKeysAndCertificateResponse = None
        self.createCertificateFromCsrResponse = None
        self.registerThingResponse = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    def _build_mqtt_connection(self):
        proxy_options = None
        if self.config.proxy_host and self.config.proxy_port:
            proxy_options = http.HttpProxyOptions(
                host_name=self.config.proxy_host,
                port=self.config.proxy_port
            )

        return mqtt_connection_builder.mtls_from_path(
            endpoint=self.config.endpoint,
            port=self.config.port,
            cert_filepath=self.config.claim_cert,
            pri_key_filepath=self.config.claim_key,
            ca_filepath=self.config.ca,
            client_id=self.config.thing_name,
            clean_session=False,
            keep_alive_secs=30,
            http_proxy_options=proxy_options,
            on_connection_interrupted=self.on_connection_interrupted,
            on_connection_resumed=self.on_connection_resumed,
        )

    def on_connection_interrupted(self, connection, error, **kwargs):
        logger.warning("MQTT connection interrupted: %s", error)

    def on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        logger.info("MQTT connection resumed. Return code: %s, Session present: %s",
                    return_code, session_present)
        if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
            resubscribe_future, _ = connection.resubscribe_existing_topics()
            resubscribe_future.add_done_callback(self.on_resubscribe_complete)

    def on_resubscribe_complete(self, future):
        result = future.result()
        logger.info("Resubscribe result: %s", result)
        for topic, qos in result['topics']:
            if qos is None:
                raise RuntimeError(f"Server rejected resubscribe to topic: {topic}")

    def connect(self):
        try:
            self.identity_client = iotidentity.IotIdentityClient(self.mqtt_connection)
            connected_future = self.mqtt_connection.connect()
            connected_future.result()
            logger.info("Connected to %s with client ID '%s'",
                        self.config.endpoint, self.config.thing_name)
        except Exception as e:
            logger.error(f"Failed to create identity client: {e}")
            raise RuntimeError("Failed to create identity client") from e

    def provision(self) -> FleetProvisioningResult:
        self.subscribe()

        if self.config.csr is None:
            self.identity_client.publish_create_keys_and_certificate(
                iotidentity.CreateKeysAndCertificateRequest(),
                mqtt.QoS.AT_LEAST_ONCE
            ).result()

            if not self.wait_for_response("createKeysAndCertificateResponse"):
                raise TimeoutError("CreateKeysAndCertificateResponse timeout")

            ownership_token = self.createKeysAndCertificateResponse.certificate_ownership_token
        else:
            with open(self.config.csr, 'r') as f:
                csr = f.read()

            self.identity_client.publish_create_certificate_from_csr(
                iotidentity.CreateCertificateFromCsrRequest(certificate_signing_request=csr),
                mqtt.QoS.AT_LEAST_ONCE
            ).result()

            if not self.wait_for_response("createCertificateFromCsrResponse"):
                raise TimeoutError("CreateCertificateFromCsrResponse timeout")

            ownership_token = self.createCertificateFromCsrResponse.certificate_ownership_token

        register_request = iotidentity.RegisterThingRequest(
            template_name=self.config.template_name,
            certificate_ownership_token=ownership_token,
            parameters=self.config.template_parameters
        )
        self.identity_client.publish_register_thing(
            register_request, mqtt.QoS.AT_LEAST_ONCE).result()

        if not self.wait_for_response("registerThingResponse", timeout=20):
            raise TimeoutError("RegisterThingResponse timeout")

        logger.info("Fleet provisioning succeeded: %s", self.registerThingResponse)

        return FleetProvisioningResult(
            thing_name=self.registerThingResponse.thing_name,
            certificate_id=(
                self.createKeysAndCertificateResponse.certificate_id
                if self.config.csr is None and self.createKeysAndCertificateResponse else
                self.createCertificateFromCsrResponse.certificate_id
            ),
            certificate_pem=(
                self.createKeysAndCertificateResponse.certificate_pem
                if self.config.csr is None and self.createKeysAndCertificateResponse else
                self.createCertificateFromCsrResponse.certificate_pem
            ),
            private_key=(
                self.createKeysAndCertificateResponse.private_key
                if self.config.csr is None and self.createKeysAndCertificateResponse else
                None
            )
        )

    def subscribe(self):
        client = self.identity_client

        if self.config.csr is None:
            req = iotidentity.CreateKeysAndCertificateSubscriptionRequest()
            accepted_future, _ = client.subscribe_to_create_keys_and_certificate_accepted(
                req, mqtt.QoS.AT_LEAST_ONCE, self.cb_create_keys_accepted)
            accepted_future.result()
            rejected_future, _ = client.subscribe_to_create_keys_and_certificate_rejected(
                req, mqtt.QoS.AT_LEAST_ONCE, self.cb_create_keys_rejected)
            rejected_future.result()
        else:
            req = iotidentity.CreateCertificateFromCsrSubscriptionRequest()
            accepted_future, _ = client.subscribe_to_create_certificate_from_csr_accepted(
                req, mqtt.QoS.AT_LEAST_ONCE, self.cb_create_csr_accepted)
            accepted_future.result()
            rejected_future, _ = client.subscribe_to_create_certificate_from_csr_rejected(
                req, mqtt.QoS.AT_LEAST_ONCE, self.cb_create_csr_rejected)
            rejected_future.result()

        req = iotidentity.RegisterThingSubscriptionRequest(template_name=self.config.template_name)
        accepted_future, _ = client.subscribe_to_register_thing_accepted(
            req, mqtt.QoS.AT_LEAST_ONCE, self.cb_register_thing_accepted)
        accepted_future.result()
        rejected_future, _ = client.subscribe_to_register_thing_rejected(
            req, mqtt.QoS.AT_LEAST_ONCE, self.cb_register_thing_rejected)
        rejected_future.result()

    def disconnect(self):
        self.mqtt_connection.disconnect().result()
        logger.info("Disconnected from MQTT broker.")

    def cb_create_keys_accepted(self, resp):
        self.createKeysAndCertificateResponse = resp
        logger.info("CreateKeysAndCertificate accepted")

    def cb_create_keys_rejected(self, err):
        raise RuntimeError(f"CreateKeysAndCertificate rejected: {err}")

    def cb_create_csr_accepted(self, resp):
        self.createCertificateFromCsrResponse = resp
        logger.info("CreateCertificateFromCsr accepted")

    def cb_create_csr_rejected(self, err):
        raise RuntimeError(f"CreateCertificateFromCsr rejected: {err}")

    def cb_register_thing_accepted(self, resp):
        self.registerThingResponse = resp
        logger.info("RegisterThing accepted")

    def cb_register_thing_rejected(self, err):
        raise RuntimeError(f"RegisterThing rejected: {err}")

    def wait_for_response(self, attr, timeout=10):
        for _ in range(timeout):
            if getattr(self, attr) is not None:
                return True
            logger.debug("Waiting for %s...", attr)
            time.sleep(1)
        return False
