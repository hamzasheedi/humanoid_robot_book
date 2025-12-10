from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
import uuid

@dataclass
class Answer:
    id: uuid.UUID
    question_id: uuid.UUID
    content: str
    sources: List[str]
    confidence_score: float
    timestamp: datetime
    metadata: dict