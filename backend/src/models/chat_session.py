from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import uuid

@dataclass
class ChatSession:
    id: uuid.UUID
    user_id: Optional[uuid.UUID]
    session_start: datetime
    session_end: Optional[datetime]
    interaction_count: int
    metadata: dict