import asyncio
import openai
from typing import List, Optional, Dict
from datetime import datetime
import uuid
from .config import OPENAI_API_KEY
from .qdrant_service import QdrantService
from .embedding_service import EmbeddingService
from .postgres_service import PostgresService
from ..models import Question, Answer, ChatSession

openai.api_key = OPENAI_API_KEY

class RAGService:
    def __init__(self, qdrant_service: QdrantService, embedding_service: EmbeddingService, postgres_service: PostgresService):
        self.qdrant_service = qdrant_service
        self.embedding_service = embedding_service
        self.postgres_service = postgres_service

    async def get_answer(self, question_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> Optional[Answer]:
        """Generate an answer to a question using RAG approach"""
        try:
            # Create embedding for the question
            query_embedding = await self.embedding_service.create_embedding(
                question_text if not selected_text else f"{selected_text} {question_text}"
            )

            # Search for relevant content in the vector store
            relevant_contents = await self.qdrant_service.search_similar(query_embedding, limit=3)

            if not relevant_contents:
                # No relevant content found, return an appropriate response
                return Answer(
                    id=uuid.uuid4(),
                    question_id=uuid.uuid4(),  # This will be updated when saved
                    content="I couldn't find any relevant content in the textbook to answer your question.",
                    sources=[],
                    confidence_score=0.0,
                    timestamp=datetime.utcnow(),
                    metadata={"reason": "no_relevant_content_found"}
                )

            # Prepare context from retrieved content
            context_parts = []
            sources = []

            for item in relevant_contents:
                content_text = item["payload"]["text"]
                context_parts.append(content_text)
                sources.append(item["payload"]["metadata"])  # This will contain source references

            context = "\n\n".join(context_parts)

            # Create the prompt for the LLM
            prompt = f"""
            You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make up information.
            If the context does not contain enough information to answer the question, say so.

            Context: {context}

            Question: {question_text}

            Answer:
            """

            # Get the answer from the LLM
            response = await openai.ChatCompletion.acreate(
                model="gpt-3.5-turbo",  # Consider using gpt-4 for more complex questions
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.3
            )

            answer_text = response.choices[0].message['content'].strip()

            # Calculate a confidence score based on the relevance of retrieved content
            confidence_score = min(1.0, sum([item["score"] for item in relevant_contents]) / len(relevant_contents) / 0.8)

            # Return the answer
            return Answer(
                id=uuid.uuid4(),
                question_id=uuid.uuid4(),  # This will be updated when saved
                content=answer_text,
                sources=sources,
                confidence_score=confidence_score,
                timestamp=datetime.utcnow(),
                metadata={
                    "context_used": len(relevant_contents),
                    "model": "gpt-3.5-turbo"
                }
            )
        except Exception as e:
            print(f"Error in RAG service: {e}")
            raise

    async def get_context_for_selection(self, selected_text: str, session_id: Optional[str] = None) -> Optional[str]:
        """Get relevant context for selected text"""
        try:
            # Create embedding for the selected text
            query_embedding = await self.embedding_service.create_embedding(selected_text)

            # Search for relevant content in the vector store
            relevant_contents = await self.qdrant_service.search_similar(query_embedding, limit=5)

            if not relevant_contents:
                return "No relevant content found in the textbook for the selected text."

            # Return the context
            context_parts = []
            for item in relevant_contents:
                content_text = item["payload"]["text"]
                context_parts.append(content_text)

            return "\n\n".join(context_parts)
        except Exception as e:
            print(f"Error getting context for selection: {e}")
            raise

    async def get_answer_with_sources(self, question_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> Dict:
        """Get an answer along with sources and metadata"""
        answer = await self.get_answer(question_text, selected_text, session_id)

        return {
            "answer": answer.content,
            "sources": answer.sources,
            "confidence": answer.confidence_score,
            "session_id": session_id
        }

    async def get_conversation_context(self, session_id: str, num_previous_interactions: int = 3) -> List[Dict]:
        """Get the recent conversation context for maintaining continuity"""
        if not session_id:
            return []

        # Retrieve recent interactions from the database
        try:
            recent_history = await self.postgres_service.get_session_history(session_id)

            # Return only the requested number of previous interactions
            recent_interactions = recent_history[-num_previous_interactions:] if len(recent_history) >= num_previous_interactions else recent_history

            # Format the context for use in prompts
            formatted_context = []
            for interaction in recent_interactions:
                formatted_context.append({
                    "question": interaction.get('question', ''),
                    "answer": interaction.get('answer', ''),
                    "timestamp": interaction.get('timestamp')
                })

            return formatted_context
        except Exception as e:
            print(f"Error retrieving conversation context: {e}")
            return []

    async def get_answer_with_context(self, question_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> Dict:
        """Generate an answer while maintaining conversation context"""
        try:
            # Get conversation context if available
            conversation_context = []
            if session_id:
                conversation_context = await self.get_conversation_context(session_id)

            # Create embedding for the question
            query_text = question_text
            if selected_text:
                query_text = f"{selected_text} {question_text}"

            query_embedding = await self.embedding_service.create_embedding(query_text)

            # Search for relevant content in the vector store
            relevant_contents = await self.qdrant_service.search_similar(query_embedding, limit=3)

            if not relevant_contents:
                # No relevant content found, return an appropriate response
                return {
                    "answer": "I couldn't find any relevant content in the textbook to answer your question.",
                    "sources": [],
                    "confidence": 0.0,
                    "session_id": session_id,
                    "reason": "no_relevant_content_found"
                }

            # Prepare context from retrieved content
            context_parts = []
            sources = []

            for item in relevant_contents:
                content_text = item["payload"]["text"]
                context_parts.append(content_text)
                sources.append(item["payload"]["metadata"])

            # Build the full context including conversation history
            full_context = "\n\n".join(context_parts)

            # Include recent conversation history if available
            if conversation_context:
                history_text = "\n\nPrevious conversation:\n"
                for i, ctx in enumerate(conversation_context):
                    history_text += f"Q{i+1}: {ctx['question']}\nA{i+1}: {ctx['answer']}\n"
                full_context += history_text

            # Create the prompt for the LLM
            prompt = f"""
            You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make up information.
            If the context does not contain enough information to answer the question, say so.
            When the user asks a follow-up question, make sure to refer to the context of the previous questions in this session.

            Textbook Context: {full_context}

            Question: {question_text}

            Answer:
            """

            # Get the answer from the LLM
            response = await openai.ChatCompletion.acreate(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.3
            )

            answer_text = response.choices[0].message['content'].strip()

            # Calculate a confidence score based on the relevance of retrieved content
            avg_relevance = sum([item["score"] for item in relevant_contents]) / len(relevant_contents)
            confidence_score = min(1.0, avg_relevance / 0.8)

            # Return the answer with conversation context
            return {
                "answer": answer_text,
                "sources": sources,
                "confidence": confidence_score,
                "session_id": session_id,
                "conversation_context_used": len(conversation_context) > 0
            }
        except Exception as e:
            print(f"Error in RAG service with context: {e}")
            raise

    async def get_answer_for_advanced_query(self, question_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> Dict:
        """Generate an answer for more complex or technical questions with enhanced context"""
        try:
            # Create embedding for the question
            query_embedding = await self.embedding_service.create_embedding(
                question_text if not selected_text else f"{selected_text} {question_text}"
            )

            # Search for relevant content in the vector store with a higher limit for complex queries
            relevant_contents = await self.qdrant_service.search_similar(query_embedding, limit=5)

            if not relevant_contents:
                # No relevant content found, return an appropriate response
                return {
                    "answer": "I couldn't find any relevant content in the textbook to answer your question.",
                    "sources": [],
                    "confidence": 0.0,
                    "session_id": session_id,
                    "reason": "no_relevant_content_found"
                }

            # Prepare context from retrieved content with more detailed information
            context_parts = []
            sources = []

            for item in relevant_contents:
                content_text = item["payload"]["text"]
                metadata = item["payload"]["metadata"]

                # Add more detailed context including source information
                detailed_context = f"Section: {metadata.get('title', 'Unknown')}\nModule: {metadata.get('module', 'Unknown')}\nContent: {content_text}"
                context_parts.append(detailed_context)

                sources.append({
                    "title": metadata.get("title", ""),
                    "module": metadata.get("module", ""),
                    "section": metadata.get("section", ""),
                    "page_reference": metadata.get("page_reference", ""),
                    "relevance_score": item["score"]
                })

            context = "\n\n---\n\n".join(context_parts)

            # Create a more detailed prompt for complex/technical questions
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make up information.
            If the context does not contain enough information to answer the question, clearly state so.
            Provide detailed explanations, examples, and be precise in your response.
            When citing specific information, reference the textbook section.
            If asked about concepts that are related but not directly answered by the context,
            explain what can be derived from the provided information.

            Context: {context}

            Question: {question_text}

            Answer:
            """

            # Get the answer from the LLM with more tokens for detailed responses
            response = await openai.ChatCompletion.acreate(
                model="gpt-4",  # Use GPT-4 for more complex technical questions
                messages=[{"role": "user", "content": prompt}],
                max_tokens=1000,  # More tokens for detailed technical explanations
                temperature=0.2   # Lower temperature for more factual, less creative responses
            )

            answer_text = response.choices[0].message['content'].strip()

            # Calculate a confidence score based on the relevance of retrieved content
            avg_relevance = sum([item["score"] for item in relevant_contents]) / len(relevant_contents)
            confidence_score = min(1.0, avg_relevance / 0.8)

            # Assess educational value of the response
            educational_assessment = await self.assess_educational_value(answer_text)

            # Return the detailed response
            return {
                "answer": answer_text,
                "sources": sources,
                "confidence": confidence_score,
                "session_id": session_id,
                "context_used": len(relevant_contents),
                "model_used": "gpt-4",
                "educational_value": educational_assessment
            }
        except Exception as e:
            print(f"Error in RAG service for advanced query: {e}")
            raise

    async def assess_educational_value(self, answer_text: str) -> Dict:
        """Assess the educational value of a response"""
        try:
            # Use a simple prompt to evaluate educational value
            # In a production system, you might want to use more sophisticated methods
            evaluation_prompt = f"""
            Evaluate the educational value of the following response on a scale of 1-5.
            Consider these factors:
            1. Clarity: Is the answer clear and easy to understand?
            2. Accuracy: Does the answer provide accurate information?
            3. Relevance: Does the answer directly address the question?
            4. Detail: Does the answer provide sufficient detail?
            5. Examples: Does the answer include examples or practical applications?

            Provide your evaluation in this format:
            - Score: [1-5]
            - Strengths: [List key strengths]
            - Areas for improvement: [List areas that could be improved]
            - Suggestions: [Specific suggestions for improvement]

            Response to evaluate:
            {answer_text}
            """

            response = await openai.ChatCompletion.acreate(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": evaluation_prompt}],
                max_tokens=500,
                temperature=0.2
            )

            evaluation_text = response.choices[0].message['content'].strip()

            # For now, return a basic assessment
            # A more sophisticated implementation would parse the evaluation_text
            # to extract specific scores and feedback

            # Simple heuristic: answers with more than 100 characters have more educational value
            length_score = min(5, max(1, len(answer_text) // 100))

            return {
                "overall_score": length_score,
                "heuristic_indicators": {
                    "length_score": length_score,
                    "has_examples": "example" in answer_text.lower(),
                    "has_explanation": "because" in answer_text.lower() or "since" in answer_text.lower()
                },
                "evaluation_details": evaluation_text
            }
        except Exception as e:
            print(f"Error assessing educational value: {e}")
            return {
                "overall_score": 3,  # Default medium score
                "heuristic_indicators": {"error": str(e)},
                "evaluation_details": "Could not assess educational value due to an error"
            }

    async def get_citations_for_answer(self, question_text: str, answer_text: str) -> List[Dict]:
        """Extract specific citations from an answer back to textbook content"""
        try:
            # Create embeddings for the answer to find highly relevant content
            answer_embedding = await self.embedding_service.create_embedding(answer_text)

            # Also create embeddings for the question to ensure context
            question_embedding = await self.embedding_service.create_embedding(question_text)

            # For citation purposes, we'll search for content highly related to the answer
            relevant_contents = await self.qdrant_service.search_similar(answer_embedding, limit=5)

            citations = []
            for item in relevant_contents:
                metadata = item["payload"]["metadata"]
                citations.append({
                    "text": item["payload"]["text"][:200] + "...",  # First 200 chars for preview
                    "title": metadata.get("title", ""),
                    "module": metadata.get("module", ""),
                    "section": metadata.get("section", ""),
                    "page_reference": metadata.get("page_reference", ""),
                    "relevance_score": item["score"]
                })

            return citations
        except Exception as e:
            print(f"Error getting citations: {e}")
            return []