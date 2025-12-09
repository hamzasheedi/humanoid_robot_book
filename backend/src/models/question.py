from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import uuid

@dataclass
class Question:
    id: uuid.UUID
    content: str
    source_context: str
    user_id: Optional[uuid.UUID]
    session_id: uuid.UUID
    timestamp: datetime
    metadata: dict