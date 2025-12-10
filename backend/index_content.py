"""
Script to index textbook content into Qdrant
This script reads markdown files from the docs directory and indexes them into the Qdrant vector database
"""
import os
import asyncio
from pathlib import Path
import uuid
from typing import List, Dict
import os
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import required modules from the project
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), "."))

from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.config import COHERE_API_KEY

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def read_markdown_files(docs_path: Path) -> List[Dict]:
    """Read all markdown files from the docs directory and extract content with metadata"""
    content_list = []

    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata from file path
            relative_path = md_file.relative_to(docs_path)
            path_parts = relative_path.parts

            metadata = {
                "title": path_parts[-1].replace('.md', '').replace('-', ' ').title(),
                "module": path_parts[0] if len(path_parts) > 1 else "general",
                "section": path_parts[1] if len(path_parts) > 2 else (path_parts[0] if len(path_parts) > 1 else "general"),
                "file_path": str(relative_path),
                "url": f"/docs/{str(relative_path).replace('.md', '')}"
            }

            content_list.append({
                "id": str(uuid.uuid5(uuid.NAMESPACE_DNS, str(md_file))),
                "content": content,
                "metadata": metadata
            })

            logger.info(f"Processed file: {relative_path}")

        except Exception as e:
            logger.error(f"Error processing file {md_file}: {e}")

    return content_list

def index_textbook_content():
    """Index all textbook content into Qdrant"""
    logger.info("Starting textbook content indexing...")

    # Initialize services
    import asyncio
    import nest_asyncio
    nest_asyncio.apply()

    async def init_services():
        qdrant_service = QdrantService()
        await qdrant_service.connect()
        embedding_service = EmbeddingService(qdrant_service)
        return qdrant_service, embedding_service

    qdrant_service, embedding_service = asyncio.run(init_services())

    # Read markdown content from docs directory
    docs_path = Path(os.path.join(os.path.dirname(__file__), "..", "..", "docs"))
    logger.info(f"Reading markdown files from: {docs_path}")

    if not docs_path.exists():
        logger.error(f"Docs directory does not exist: {docs_path}")
        return

    markdown_contents = read_markdown_files(docs_path)
    logger.info(f"Found {len(markdown_contents)} markdown files to index")

    if not markdown_contents:
        logger.warning("No markdown files found to index")
        return

    async def index_content():
        # Process and index content
        for i, item in enumerate(markdown_contents):
            try:
                logger.info(f"Indexing ({i+1}/{len(markdown_contents)}): {item['metadata']['title']}")

                # Index content using the embedding service
                await embedding_service.index_textbook_content(
                    content_id=item["id"],
                    text=item["content"],
                    metadata=item["metadata"]
                )

            except Exception as e:
                logger.error(f"Error indexing content {item['id']}: {e}")
        logger.info("Textbook content indexing completed successfully!")

    asyncio.run(index_content())

if __name__ == "__main__":
    index_textbook_content()