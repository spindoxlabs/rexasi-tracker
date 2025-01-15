from typing import List, Tuple
from pydantic import BaseModel as PydanticBaseModel
from filterpy.kalman import KalmanFilter

class BaseModel(PydanticBaseModel):
    class Config:
        arbitrary_types_allowed = True

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
