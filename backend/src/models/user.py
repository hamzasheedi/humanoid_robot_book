from pydantic import BaseModel
from typing import Optional, Union
from datetime import datetime
from uuid import UUID


class UserBase(BaseModel):
    email: str


class UserCreate(UserBase):
    password: str
    os: Optional[str] = None
    cpu: Optional[str] = None
    gpu: Optional[str] = None
    ram_gb: Optional[int] = None
    programming_experience: Optional[str] = None  # beginner, intermediate, advanced, expert
    robotics_experience: Optional[str] = None    # none, beginner, intermediate, advanced
    development_environment: Optional[str] = None
    primary_language: Optional[str] = None
    learning_goals: Optional[list] = []

    class Config:
        # Allow the model to handle None values properly for learning_goals
        arbitrary_types_allowed = True


class UserUpdate(BaseModel):
    os: Optional[str] = None
    cpu: Optional[str] = None
    gpu: Optional[str] = None
    ram_gb: Optional[int] = None
    programming_experience: Optional[str] = None
    robotics_experience: Optional[str] = None
    development_environment: Optional[str] = None
    primary_language: Optional[str] = None
    learning_goals: Optional[list] = None

    class Config:
        arbitrary_types_allowed = True


class User(UserBase):
    id: Union[str, UUID]  # Can handle both UUID objects and string representations
    created_at: datetime
    updated_at: datetime
    os: Optional[str] = None
    cpu: Optional[str] = None
    gpu: Optional[str] = None
    ram_gb: Optional[int] = None
    programming_experience: Optional[str] = None
    robotics_experience: Optional[str] = None
    development_environment: Optional[str] = None
    primary_language: Optional[str] = None
    learning_goals: Optional[list] = []

    class Config:
        from_attributes = True
        arbitrary_types_allowed = True
        json_encoders = {
            UUID: str
        }