import asyncio
import openai
from typing import List
from .config import OPENAI_API_KEY
from .qdrant_service import QdrantService
import logging

openai.api_key = OPENAI_API_KEY

class EmbeddingService:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.model = "text-embedding-ada-002"  # OpenAI's recommended embedding model
    
    async def create_embedding(self, text: str) -> List[float]:
        """Create an embedding for a single text"""
        try:
            response = await openai.Embedding.acreate(
                input=text,
                model=self.model
            )
            return response['data'][0]['embedding']
        except Exception as e:
            print(f"Error creating embedding: {e}")
            raise
    
    async def create_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for a batch of texts"""
        try:
            response = await openai.Embedding.acreate(
                input=texts,
                model=self.model
            )
            return [item['embedding'] for item in response['data']]
        except Exception as e:
            print(f"Error creating embeddings batch: {e}")
            raise
    
    async def index_textbook_content(self, content_id: str, text: str, metadata: dict = None):
        """Index a single piece of textbook content with its embedding"""
        if not metadata:
            metadata = {}
        
        embedding = await self.create_embedding(text)
        payload = {
            "text": text,
            "metadata": metadata
        }
        await self.qdrant_service.add_embedding(content_id, embedding, payload)
    
    async def index_textbook_content_batch(self, content_ids: List[str], texts: List[str], metadata_list: List[dict] = None):
        """Index a batch of textbook content with their embeddings"""
        if metadata_list is None:
            metadata_list = [{}] * len(texts)
        
        if len(content_ids) != len(texts) or len(content_ids) != len(metadata_list):
            raise ValueError("content_ids, texts, and metadata_list must have the same length")
        
        embeddings = await self.create_embeddings_batch(texts)
        payloads = [
            {"text": text, "metadata": metadata}
            for text, metadata in zip(texts, metadata_list)
        ]
        
        await self.qdrant_service.add_embeddings_batch(content_ids, embeddings, payloads)
    
    async def search_relevant_content(self, query: str, limit: int = 3) -> List[dict]:
        """Search for content relevant to a query"""
        embedding = await self.create_embedding(query)
        return await self.qdrant_service.search_similar(embedding, limit)
    
    async def process_textbook_chapter(self, chapter_title: str, chapter_content: str, module: str, section: str):
        """Process a single textbook chapter by breaking it into chunks and indexing each chunk"""
        # Break chapter into chunks (simple paragraph-based approach for now)
        paragraphs = [p.strip() for p in chapter_content.split('\n\n') if p.strip()]
        
        content_ids = []
        texts = []
        metadata_list = []
        
        for i, paragraph in enumerate(paragraphs):
            if len(paragraph) > 50:  # Only index paragraphs with substantial content
                content_id = f"{module}_{section}_{i}"
                content_ids.append(content_id)
                texts.append(paragraph)
                metadata_list.append({
                    "title": chapter_title,
                    "module": module,
                    "section": section,
                    "paragraph_index": i
                })
        
        if content_ids:
            await self.index_textbook_content_batch(content_ids, texts, metadata_list)
            print(f"Indexed {len(content_ids)} chunks for chapter: {chapter_title}")
        
        return content_ids