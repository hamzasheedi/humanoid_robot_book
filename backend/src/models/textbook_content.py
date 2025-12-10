from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import uuid

@dataclass
class TextbookContent:
    id: uuid.UUID
    title: str
    content: str
    module: str
    section: str
    page_reference: str
    embedding_id: str
    created_at: datetime
    updated_at: datetime