from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import uuid

@dataclass
class UserInteractionLog:
    id: uuid.UUID
    question_id: uuid.UUID
    answer_id: uuid.UUID
    accuracy_rating: Optional[int]
    useful: bool
    feedback_text: Optional[str]
    timestamp: datetime
    metadata: dict