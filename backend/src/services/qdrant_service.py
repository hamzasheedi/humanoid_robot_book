import asyncio
from typing import List, Dict, Optional
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from ..config import QDRANT_URL, QDRANT_API_KEY

class QdrantService:
    def __init__(self):
        self.client: Optional[AsyncQdrantClient] = None
        self.collection_name = "textbook_content"
    
    async def connect(self):
        """Initialize Qdrant client connection"""
        try:
            if QDRANT_API_KEY:
                self.client = AsyncQdrantClient(
                    url=QDRANT_URL,
                    api_key=QDRANT_API_KEY,
                    prefer_grpc=False
                )
            else:
                self.client = AsyncQdrantClient(host=QDRANT_URL)
            
            print("Qdrant client connected successfully")
        except Exception as e:
            print(f"Error connecting to Qdrant: {e}")
            raise
    
    async def setup_collection(self):
        """Create or verify collection exists with proper vector configuration"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create collection with 1536 dimensions (for OpenAI embeddings)
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
                )
                print(f"Created Qdrant collection: {self.collection_name}")
            else:
                print(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            print(f"Error setting up Qdrant collection: {e}")
            raise

    async def add_embedding(self, content_id: str, embedding: List[float], payload: Dict):
        """Add a single embedding to the collection"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        try:
            await self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=content_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
        except Exception as e:
            print(f"Error adding embedding: {e}")
            raise

    async def add_embeddings_batch(self, content_ids: List[str], embeddings: List[List[float]], payloads: List[Dict]):
        """Add multiple embeddings to the collection"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        if len(content_ids) != len(embeddings) or len(content_ids) != len(payloads):
            raise ValueError("content_ids, embeddings, and payloads lists must have the same length")
        
        try:
            points = [
                models.PointStruct(
                    id=content_id,
                    vector=embedding,
                    payload=payload
                )
                for content_id, embedding, payload in zip(content_ids, embeddings, payloads)
            ]
            
            await self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            print(f"Error adding embeddings batch: {e}")
            raise

    async def search_similar(self, query_embedding: List[float], limit: int = 3) -> List[Dict]:
        """Search for similar content based on embedding"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        try:
            results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )
            
            return [
                {
                    "id": result.id,
                    "payload": result.payload,
                    "score": result.score
                }
                for result in results
            ]
        except Exception as e:
            print(f"Error searching for similar content: {e}")
            return []
    
    async def get_content_by_id(self, content_id: str) -> Optional[Dict]:
        """Get content by its ID"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        try:
            records = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id]
            )
            
            if records:
                return {
                    "id": records[0].id,
                    "payload": records[0].payload
                }
            return None
        except Exception as e:
            print(f"Error retrieving content by ID: {e}")
            return None
    
    async def delete_content_by_id(self, content_id: str):
        """Delete content by its ID"""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")
        
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
        except Exception as e:
            print(f"Error deleting content by ID: {e}")
            raise