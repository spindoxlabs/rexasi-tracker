from typing import List, Tuple, Optional, Any, Set

import numpy as np
from pydantic import Field
from pydantic import BaseModel as PydanticBaseModel
from filterpy.kalman import KalmanFilter


class BaseModel(PydanticBaseModel):
    class Config:
        arbitrary_types_allowed = True

class CenterData(BaseModel):
    pixels: Optional[List[Tuple[int, ...]]] = []
    coordinates: Optional[List[Tuple[float, ...]]] = []
    fused_coordinates: Optional[List[Tuple[float, ...]]] = []

class Keypoints(BaseModel):
    pixels: Optional[List[List[List[float]]]] = []
    coordinates: Optional[List[List[List[float]]]] = []

class SensorData(BaseModel):
    frame_number: int  # todo: watch out to this
    idx: int
    sensor_type: str = "rgb"
    timestamp: int
    detections: Optional[Keypoints] = Keypoints()
    centers: Optional[CenterData] = CenterData()
    confidence: Optional[List[float]] = []

class SensorTrackedData(SensorData):
    identities: Optional[List[int]] = []
    dead_identities: Optional[List[int]] = []
    fused_identities: Optional[List[int]] = []

class SensorTrack(BaseModel):
    identity: int = 0
    sensor_id: int = 0
    center: Tuple[float, ...] = ()
    fused_track_ref: int = 0
    dead: bool = False
    timestamp: int = 0
    confidence: float = 1.0

class KalmanFilterData(BaseModel):
    kf: KalmanFilter = None
    last_ts: float = 0.0

class AssociationTrack(BaseModel):
    identity: int = 0
    center: Tuple[float, ...] = ()
    current_sensor_id: int = 0
    timestamp: int = 0
    sensor_track_refs: List[SensorTrack] = []
    kalman_filter: KalmanFilterData = KalmanFilterData()
    confidence: float = 1.0
