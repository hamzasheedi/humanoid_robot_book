from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class PersonalizationBase(BaseModel):
    user_id: str


class PersonalizationCreate(PersonalizationBase):
    content_difficulty: Optional[str] = "adaptive"  # adaptive, beginner, intermediate, advanced
    preferred_examples: Optional[List[str]] = []
    response_complexity: Optional[str] = "balanced"  # simple, balanced, detailed
    interaction_style: Optional[str] = "guided"  # guided, exploratory, problem-solving
    learning_pace: Optional[str] = "moderate"  # slow, moderate, fast
    personalization_preferences: Optional[Dict[str, Any]] = {}


class PersonalizationUpdate(BaseModel):
    content_difficulty: Optional[str] = None
    preferred_examples: Optional[List[str]] = None
    response_complexity: Optional[str] = None
    interaction_style: Optional[str] = None
    learning_pace: Optional[str] = None
    personalization_preferences: Optional[Dict[str, Any]] = None


class Personalization(BaseModel):
    id: str
    created_at: datetime
    updated_at: datetime
    content_difficulty: str
    preferred_examples: List[str]
    response_complexity: str
    interaction_style: str
    learning_pace: str
    personalization_preferences: Dict[str, Any]

    class Config:
        from_attributes = True