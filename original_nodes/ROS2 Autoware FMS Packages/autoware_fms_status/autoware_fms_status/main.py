import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import os
import json
import pathlib
from base64 import b64decode

from Crypto.PublicKey import RSA
from authlib.integrations.requests_client import OAuth2Session
from authlib.oauth2.rfc7523 import PrivateKeyJWT

import asyncio
import websockets
import time
import requests

from autoware_fms_msgs.msg import Schedule, Task, Place, Tag, PlaceArray
from builtin_interfaces.msg import Time
import dateutil.parser
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class FMSTokenSession:
    def __init__(self, client_id: str, subject_id: str, private_key: str, logger=None):
        self.token_endpoint = 'https://auth.web.auto/v2/internal/token'
        self.client_id = client_id
        self.jwt_key = RSA.importKey(b64decode(private_key)).exportKey()
        self.jwt_key_private = PrivateKeyJWT(claims={'sub': subject_id})

        self.logger_ = logger

        # retrieve token every 300 seconds
        self.token = None
        self.token_update_time = None
        self.update_token()

    def update_token(self) -> None:
        if self.logger_ is not None:
            self.logger_.info("Fetching token information ...")
        session = OAuth2Session(self.client_id, self.jwt_key, self.jwt_key_private, token_endpoint=self.token_endpoint, grant_type='client_credentials')
        try:
            self.token = session.fetch_token()
            self.token_update_time = self.token['expires_at']
        except Exception as e:
            self.logger_.error(f"Exception caught while fetching token: {e}")

    def get_token(self) -> str:
        if (self.token is None) or (self.token_update_time is None):
            self.update_token()
        if (self.token_update_time - time.time()) <= 5.0: # refresh if within 5 seconds
            self.update_token()
        return self.token['access_token']

def iso_timestamp_to_ros(iso_timestamp: str) -> Time:
    datetime_obj = dateutil.parser.isoparse(iso_timestamp)
    output_msg = Time()
    output_msg.sec = int(datetime_obj.timestamp())
    output_msg.nanosec = datetime_obj.microsecond * 1000 # convert from microsecond to nanosecond
    return output_msg

class MainNode(Node):
    def __init__(self):
        super().__init__('autoware_fms_status')
        self.logger_ = self.get_logger()

        self.oa_client_id_ = self.declare_parameter('oauth2_client_id', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))
        self.oa_subject_id = self.declare_parameter('oauth2_subject_id', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))
        self.oa_private_key_ = self.declare_parameter('oauth2_private_key', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))

        self.vehicle_id_param_ = self.declare_parameter('vehicle_id', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))
        self.vehicle_id_ = self.vehicle_id_param_.get_parameter_value().string_value
        self.project_id_param_ = self.declare_parameter('project_id', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))
        self.project_id_ = self.project_id_param_.get_parameter_value().string_value
        self.environment_id_param_ = self.declare_parameter('environment_id', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=""))
        self.environment_id_ = self.environment_id_param_.get_parameter_value().string_value

        self.logger_.info("Attempting to retrieve token via WebAutoAuth ...")
        self.fms_session_ = FMSTokenSession(
            self.oa_client_id_.get_parameter_value().string_value, 
            self.oa_subject_id.get_parameter_value().string_value, 
            self.oa_private_key_.get_parameter_value().string_value
        )

        qos_profile = QoSProfile( # allow subscribers to receive last message, even if not actively publishing
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.place_array_publisher_ = self.create_publisher(PlaceArray, '~/places', qos_profile=qos_profile)
        self.schedule_publisher_ = self.create_publisher(Schedule, '~/current_schedule', qos_profile=qos_profile)

        # retrieve & publish places list
        self.logger_.info("Attempting to retrieve list of bus stops ...")
        place_array_msg = self.retrieve_places()
        self.place_array_publisher_.publish(place_array_msg)

        self.logger_.info("Starting main websocket link ...")
        asyncio.run(self.websocket_link())

    def decode_task(self, json: dict) -> Task:
        msg = Task()

        msg.task_id = json['task_id']

        task_type = 0
        match json['task_type']:
            case 'move':
                task_type = Task.MOVE
        msg.task_type = task_type

        msg.is_reserved = json['is_reserved']

        status = 0
        match json['status']:
            case 'todo':
                status = Task.TODO
            case 'doing':
                status = Task.DOING
            case 'done':
                status = Task.DONE
            case 'aborting':
                status = Task.ABORTING
            case 'aborted':
                status = Task.ABORTED
            case 'disabled':
                status = Task.DISABLED
        msg.status = status

        origin_msg = Place()
        origin_msg.lat = json['origin']['location']['lat']
        origin_msg.lon = json['origin']['location']['lng']
        msg.origin = origin_msg

        destination_msg = Place()
        destination_msg.point_id = json['destination']['point_id']
        destination_msg.name = json['destination']['name']
        destination_msg.lat = json['destination']['location']['lat']
        destination_msg.lon = json['destination']['location']['lng']
        msg.destination = destination_msg

        msg.plan_start_time = iso_timestamp_to_ros(json['plan_start_time'])
        msg.plan_end_time = iso_timestamp_to_ros(json['plan_end_time'])
        msg.duration_sec = json['duration_sec']

        if 'route_ids' in json:
            msg.route_ids = json['route_ids']

        if 'actual_start_time' in json:
            msg.actual_start_time = iso_timestamp_to_ros(json['actual_start_time'])

        return msg

    def decode_schedule(self, json: dict) -> Schedule:
        msg = Schedule()

        msg.schedule_id = json['schedule_id']
        msg.project_id = json['project_id']
        msg.environment_id = json['environment_id']

        schedule_type = 0
        match json['schedule_type']:
            case 'one-way':
                schedule_type = Schedule.ONEWAY
            case 'cyclic':
                schedule_type = Schedule.CYCLIC
            case 'maintenance':
                schedule_type = Schedule.MAINTENANCE
        msg.schedule_type = schedule_type

        msg.is_reserved = json['is_reserved']
        msg.vehicle_id = json['vehicle_id']

        status = 0
        match json['status']:
            case 'doing':
                status = Schedule.DOING
            case 'cancelling':
                status = Schedule.CANCELLING
            case 'aborting':
                status = Schedule.ABORTING
        msg.status = status

        msg.plan_start_time = iso_timestamp_to_ros(json['plan_start_time'])
        msg.delta_sec = json['delta_sec']

        tasks = []
        for task in json['tasks']:
            tasks.append(self.decode_task(task))
        msg.tasks = tasks

        msg.created_at = iso_timestamp_to_ros(json['created_at'])
        msg.updated_at = iso_timestamp_to_ros(json['updated_at'])

        if 'plan_end_time' in json:
            msg.plan_end_time = iso_timestamp_to_ros(json['plan_end_time'])

        msg.actual_start_time = iso_timestamp_to_ros(json['actual_start_time'])

        if 'duration_sec' in json:
            msg.duration_sec = json['duration_sec']

        if 'tags' in json:
            tags = []
            for tag in json['tags']:
                tag_msg = Tag()
                tag_msg.key = tag['key']
                tag_msg.value = tag['value']
                tags.append(tag_msg)
            msg.tags = tags

        return msg

    def publish_schedule(self, json: dict) -> None:
        schedule_msg = self.decode_schedule(json)
        self.schedule_publisher_.publish(schedule_msg)

    async def websocket_link(self):
        fms_ws_uri = f"wss://fms.web.auto/api/v1/ws?project_id={self.project_id_}&token={self.fms_session_.get_token()}"
        self.logger_.info(f"Establishing WebSockets link to {fms_ws_uri} ...")
        async with websockets.connect(fms_ws_uri) as ws:
            for channel in ["vehicleActiveSchedule"]:
                self.logger_.info(f"Attempting to subscribe to {channel} via WebSockets ...")
                subscribe_message = {
                    "token": self.fms_session_.get_token(),
                    "channel": channel,
                    "operation": "subscribe",
                    "resource_id": self.vehicle_id_
                }

                subscribe_message_json = json.dumps(subscribe_message)
                await ws.send(subscribe_message_json)

            self.logger_.info("Starting retrieval of API messages ...")
            async for api_message in ws:
                api_message_json = json.loads(api_message)
                self.publish_schedule(api_message_json)

    def retrieve_from_api(self, request_uri: str) -> dict:
        self.logger_.info(f"Retrieving {request_uri} ...")
        endpoint_url = "https://fms.web.auto/api/v1"
        req = requests.get(
            f'{endpoint_url}{request_uri}', # /projects/{self.project_id}/environments/{self.environment_id}/places
            headers={
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {self.fms_session_.get_token()}'
            }
        )
        return req.json()

    def retrieve_places(self) -> PlaceArray:
        places_api_response = self.retrieve_from_api(f"/projects/{self.project_id_}/environments/{self.environment_id_}/places")
        place_array_msg = PlaceArray()
        place_array_msg.area_map_version_id = places_api_response['area_map_version_id']

        places = []
        for place in places_api_response['places']:
            place_msg = Place()
            place_msg.point_id = place['point_id']
            place_msg.lat = place['location']['lat']
            place_msg.lon = place['location']['lng']
            place_msg.name = place['name']
            places.append(place_msg)
        place_array_msg.places = places
        return place_array_msg


def main(args=None):
    rclpy.init(args=args)

    main_node = MainNode()

    rclpy.spin(main_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
