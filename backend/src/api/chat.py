from fastapi import APIRouter, HTTPException, Depends, Request
import uuid
from typing import Optional, List, Dict
from datetime import datetime
import asyncio

from ..models import Question, Answer, ChatSession
from ..api.main import rag_service, postgres_service
from ..utils.logging import log_request, log_response, log_error
from ..utils.validation import validate_question_input

router = APIRouter()

@router.post("/ask")
async def ask_question(
    request: Request,
    question: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None
):
    """
    Submit a question to get an answer based only on textbook content
    """
    # Log request
    log_request(request, {"question_length": len(question), "session_id": session_id})
    
    # Validate input
    validation_result = validate_question_input(question)
    if not validation_result["valid"]:
        raise HTTPException(status_code=400, detail=validation_result["message"])
    
    try:
        # Create or use existing session
        if not session_id:
            session_id = await postgres_service.create_session()
        
        # Create question object
        question_obj = Question(
            id=uuid.uuid4(),
            content=question,
            source_context=selected_text or "",
            user_id=None,  # Will implement user auth later if needed
            session_id=uuid.UUID(session_id),
            timestamp=datetime.utcnow(),
            metadata={"request_type": "ask_question", "source": "api"}
        )
        
        # Get answer using RAG service
        answer_obj = await rag_service.get_answer(question, selected_text, session_id)
        
        # Complete answer object with proper IDs
        answer_obj.id = uuid.uuid4()
        answer_obj.question_id = question_obj.id
        
        # Save to database
        await postgres_service.save_answer_with_question(question_obj, answer_obj)
        
        # Prepare response
        response = {
            "answer": answer_obj.content,
            "sources": answer_obj.sources,
            "confidence": answer_obj.confidence_score,
            "session_id": session_id
        }
        
        log_response(response, session_id)
        return response
        
    except Exception as e:
        log_error(e, "ask_question")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/context")
async def get_context_from_selection(
    request: Request,
    selected_text: str,
    session_id: Optional[str] = None
):
    """
    Submit selected text to get relevant context from the textbook
    """
    # Log request
    log_request(request, {"selected_text_length": len(selected_text), "session_id": session_id})
    
    if not selected_text or len(selected_text.strip()) == 0:
        raise HTTPException(status_code=400, detail="Selected text cannot be empty")
    
    try:
        # Get context using RAG service
        context = await rag_service.get_context_for_selection(selected_text, session_id)
        
        if not context or context.startswith("No relevant content"):
            raise HTTPException(status_code=404, detail="No relevant content found in the textbook")
        
        response = {
            "context": context,
            "sources": [],  # Will add specific sources if needed
            "session_id": session_id or ""
        }
        
        log_response(response, session_id)
        return response
        
    except HTTPException:
        raise
    except Exception as e:
        log_error(e, "get_context_from_selection")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/history")
async def get_chat_history(request: Request, session_id: str):
    """
    Retrieve the history of questions and answers for a specific session
    """
    log_request(request, {"session_id": session_id})

    if not session_id:
        raise HTTPException(status_code=400, detail="Session ID is required")

    try:
        history = await postgres_service.get_session_history(session_id)

        if not history:
            # Verify session exists
            session = await postgres_service.get_session(session_id)
            if not session:
                raise HTTPException(status_code=404, detail="Session not found")

        response = {
            "session_id": session_id,
            "interactions": history
        }

        log_response(response, session_id)
        return response

    except HTTPException:
        raise
    except Exception as e:
        log_error(e, "get_chat_history")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/history/{session_id}")
async def get_session_history(session_id: str):
    """
    Retrieve the history of questions and answers for a specific session (alternative endpoint)
    """
    if not session_id:
        raise HTTPException(status_code=400, detail="Session ID is required")

    try:
        history = await postgres_service.get_session_history(session_id)

        if not history:
            # Verify session exists
            session = await postgres_service.get_session(session_id)
            if not session:
                raise HTTPException(status_code=404, detail="Session not found")

        response = {
            "session_id": session_id,
            "interactions": history
        }

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))