from pydantic import BaseModel
from typing import Optional, Union
from datetime import datetime
from uuid import UUID
from ipaddress import IPv4Address


class SessionBase(BaseModel):
    user_id: Union[str, UUID]
    session_token: str


class SessionCreate(SessionBase):
    expires_at: datetime
    ip_address: Optional[Union[str, IPv4Address]] = None
    user_agent: Optional[str] = None


class Session(SessionBase):
    id: Union[str, UUID]
    created_at: datetime
    updated_at: datetime
    expires_at: datetime
    is_active: bool
    ip_address: Optional[Union[str, IPv4Address]] = None
    user_agent: Optional[str] = None

    class Config:
        from_attributes = True
        json_encoders = {
            UUID: str,
            IPv4Address: str
        }