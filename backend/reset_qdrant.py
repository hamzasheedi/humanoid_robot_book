"""
Qdrant collection reset script
This script deletes and recreates the Qdrant collection with correct dimensions for Cohere embeddings
"""
import asyncio
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

async def reset_qdrant_collection():
    try:
        # Initialize the client
        if QDRANT_API_KEY:
            client = AsyncQdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
                prefer_grpc=False
            )
        else:
            client = AsyncQdrantClient(host=QDRANT_URL)

        print("Connected to Qdrant client")
        
        collection_name = "textbook_content"
        
        # Delete existing collection
        try:
            await client.delete_collection(collection_name=collection_name)
            print(f"Deleted existing collection: {collection_name}")
        except Exception as e:
            print(f"Collection {collection_name} might not exist yet, continuing...")
        
        # Create collection with correct dimensions (1024 for Cohere embeddings)
        await client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
        )
        
        print(f"Created new collection: {collection_name} with 1024 dimensions")
        
        # Verify the collection was created properly
        collections = await client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)
        
        if collection_exists:
            print(f"+ Collection {collection_name} successfully created with correct dimensions")
        else:
            print(f"- Failed to create collection {collection_name}")
            
    except Exception as e:
        print(f"Error resetting Qdrant collection: {e}")
        raise

if __name__ == "__main__":
    asyncio.run(reset_qdrant_collection())