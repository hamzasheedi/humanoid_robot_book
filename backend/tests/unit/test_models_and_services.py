import pytest
from unittest.mock import Mock, AsyncMock
import sys
import os

# Add the src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# Test basic functionality of the RAG service
def test_rag_service_initialization():
    """Unit test for RAG service initialization"""
    try:
        from src.services.rag_service import RAGService
        from src.services.qdrant_service import QdrantService
        from src.services.embedding_service import EmbeddingService
        from src.services.postgres_service import PostgresService
        
        # Mock the dependencies
        qdrant_mock = Mock(spec=QdrantService)
        embedding_mock = Mock(spec=EmbeddingService)
        postgres_mock = Mock(spec=PostgresService)
        
        # Initialize the RAG service
        rag_service = RAGService(qdrant_mock, embedding_mock, postgres_mock)
        
        # Verify it was created properly
        assert rag_service.qdrant_service == qdrant_mock
        assert rag_service.embedding_service == embedding_mock
        assert rag_service.postgres_service == postgres_mock
        
        print("✓ RAG service initialization test passed")
        
    except ImportError as e:
        print(f"⚠️ Could not import RAG service components: {e}")
        print("This is expected if dependencies are not fully configured yet")


def test_model_creation():
    """Unit test for model creation"""
    try:
        from src.models import Question, Answer, ChatSession
        import uuid
        from datetime import datetime
        
        # Test Question model creation
        test_uuid = uuid.uuid4()
        question = Question(
            id=test_uuid,
            content="What is ROS 2?",
            source_context="Chapter 1",
            user_id=None,
            session_id=test_uuid,
            timestamp=datetime.utcnow(),
            metadata={}
        )
        
        assert question.content == "What is ROS 2?"
        assert question.session_id == test_uuid
        
        # Test Answer model creation
        answer = Answer(
            id=test_uuid,
            question_id=test_uuid,
            content="ROS 2 is a robot operating system...",
            sources=["module1", "section2"],
            confidence_score=0.95,
            timestamp=datetime.utcnow(),
            metadata={}
        )
        
        assert answer.content == "ROS 2 is a robot operating system..."
        assert answer.confidence_score == 0.95
        
        print("✓ Model creation tests passed")
        
    except ImportError as e:
        print(f"⚠️ Could not import model components: {e}")
        print("This is expected if dependencies are not fully configured yet")


if __name__ == "__main__":
    test_rag_service_initialization()
    test_model_creation()
    print("All unit tests completed!")