import asyncio
from uuid import uuid4
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize services (similar to how they would be initialized in main backend)
from backend.src.config import OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY
from backend.src.services.qdrant_service import QdrantService
from backend.src.services.embedding_service import EmbeddingService

# Sample textbook content chunks for demonstration
TEXTBOOK_MODULES = {
    "ros2": {
        "name": "ROS 2 Fundamentals",
        "sections": {
            "introduction": "ROS 2 (Robot Operating System 2) is a set of libraries and tools that help you build robot applications...",
            "architecture": "The ROS 2 architecture consists of nodes, topics, services, actions, and parameters...",
            "nodes_topics": "Nodes are the basic unit of execution in ROS 2...",
            "services_actions": "Services provide a request/response communication pattern in ROS 2..."
        }
    },
    "gazebo_unity": {
        "name": "Digital Twin (Gazebo & Unity)",
        "sections": {
            "gazebo_basics": "Gazebo is a robotics simulator that allows you to create, test, and debug robot applications...",
            "unity_integration": "Unity provides a 3D development platform that can be integrated with ROS 2 for simulation...",
            "physics_simulation": "Accurate physics simulation is crucial for robot development and testing in virtual environments..."
        }
    },
    "nvidia_isaac": {
        "name": "NVIDIA Isaac",
        "sections": {
            "overview": "NVIDIA Isaac is a robotics platform that provides hardware and software for AI-powered robots...",
            "development_tools": "Isaac includes tools for simulation, navigation, manipulation, and perception...",
            "ai_integration": "Deep learning and computer vision capabilities are key components of the Isaac platform..."
        }
    }
}

async def load_textbook_content_to_qdrant():
    """Load textbook content into Qdrant for RAG retrieval"""
    print("Initializing Qdrant and embedding services...")
    
    # Initialize services
    qdrant_service = QdrantService()
    await qdrant_service.connect()
    
    # Connect to Qdrant and setup collection
    await qdrant_service.setup_collection()
    
    embedding_service = EmbeddingService(qdrant_service)
    
    print("Loading textbook content to Qdrant...")
    
    content_chunks = []
    
    # Process each module and section
    for module_id, module_info in TEXTBOOK_MODULES.items():
        for section_id, content in module_info["sections"].items():
            # Create content chunks (break into smaller pieces if needed)
            content_id = f"{module_id}_{section_id}_{str(uuid4())[:8]}"
            
            metadata = {
                "module_id": module_id,
                "module_name": module_info["name"],
                "section_id": section_id,
                "section_title": section_id.replace("_", " ").title()
            }
            
            content_chunks.append({
                "id": content_id,
                "text": content,
                "metadata": metadata
            })
    
    # Process chunks in batches
    batch_size = 10  # Adjust based on API limits
    total_chunks = len(content_chunks)
    
    print(f"Indexing {total_chunks} content chunks...")
    
    for i in range(0, total_chunks, batch_size):
        batch = content_chunks[i:i + batch_size]
        
        # Extract data for batch processing
        content_ids = [chunk["id"] for chunk in batch]
        texts = [chunk["text"] for chunk in batch]
        metadata_list = [chunk["metadata"] for chunk in batch]
        
        print(f"Indexing batch {i//batch_size + 1}/{(total_chunks-1)//batch_size + 1} ({len(batch)} chunks)")
        
        # Index the batch
        await embedding_service.index_textbook_content_batch(
            content_ids=content_ids,
            texts=texts,
            metadata_list=metadata_list
        )
    
    print(f"Successfully loaded {total_chunks} textbook content chunks to Qdrant!")
    print("Textbook content is now available for RAG retrieval.")
    
    # Close connections
    # Note: In a real implementation, you wouldn't close the connection here
    # as it would be managed by the main application lifecycle

if __name__ == "__main__":
    print("Starting textbook content loading process...")
    asyncio.run(load_textbook_content_to_qdrant())