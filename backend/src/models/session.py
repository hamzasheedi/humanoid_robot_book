from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class SessionBase(BaseModel):
    user_id: str
    session_token: str


class SessionCreate(SessionBase):
    expires_at: datetime
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None


class Session(SessionBase):
    id: str
    created_at: datetime
    updated_at: datetime
    expires_at: datetime
    is_active: bool
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None

    class Config:
        from_attributes = True