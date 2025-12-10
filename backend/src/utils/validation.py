def validate_question_input(question: str) -> dict:
    """Validate question input from the user"""
    if not question or len(question.strip()) == 0:
        return {
            "valid": False,
            "message": "Question cannot be empty"
        }
    
    if len(question) > 1000:  # Arbitrary limit, can be adjusted
        return {
            "valid": False,
            "message": "Question is too long (max 1000 characters)"
        }
    
    if len(question) < 3:
        return {
            "valid": False,
            "message": "Question is too short (min 3 characters)"
        }
    
    # Additional validation can be added here (e.g., check for inappropriate content)
    
    return {
        "valid": True,
        "message": "Question is valid"
    }

def validate_text_selection(selected_text: str) -> dict:
    """Validate text selection input from the user"""
    if not selected_text or len(selected_text.strip()) == 0:
        return {
            "valid": False,
            "message": "Selected text cannot be empty"
        }
    
    if len(selected_text) > 5000:  # Arbitrary limit, can be adjusted
        return {
            "valid": False,
            "message": "Selected text is too long (max 5000 characters)"
        }
    
    return {
        "valid": True,
        "message": "Selected text is valid"
    }

def validate_session_id(session_id: str) -> dict:
    """Validate session ID"""
    import uuid

    if not session_id:
        return {
            "valid": True,  # Session ID is optional in many cases
            "message": "No session ID provided (this is OK)"
        }

    try:
        uuid.UUID(session_id)
        return {
            "valid": True,
            "message": "Session ID is valid"
        }
    except ValueError:
        return {
            "valid": False,
            "message": "Session ID is not a valid UUID"
        }

def validate_advanced_question(question: str) -> dict:
    """Validate if a question is suitable for advanced processing"""
    import re

    validation_result = validate_question_input(question)
    if not validation_result["valid"]:
        return validation_result

    # Check if the question appears to be technical or advanced
    technical_indicators = [
        r'\balgorithm\b', r'\bimplementation\b', r'\barchitecture\b',
        r'\bprotocol\b', r'\bframework\b', r'\bmodel\b', r'\bsystem\b',
        r'\bperformance\b', r'\boptimizat', r'\bcomplexity\b',
        'how does', 'what is the difference', 'compare', 'contrast',
        'explain in detail', 'elaborate on'
    ]

    is_advanced = any(re.search(indicator, question, re.IGNORECASE) for indicator in technical_indicators)

    # Additional validation for advanced questions
    if len(question) > 200:
        return {
            "valid": True,
            "is_advanced": True,
            "message": "Complex question validated"
        }

    return {
        "valid": True,
        "is_advanced": is_advanced,
        "message": "Question validated"
    }

def validate_educational_content(content: str) -> dict:
    """Validate if content is educationally valuable"""
    if not content or len(content.strip()) == 0:
        return {
            "valid": False,
            "message": "Content cannot be empty"
        }

    # Check for educational value indicators
    educational_indicators = [
        r'\bexample\b', r'\bexample:', r'\bfor instance\b', r'\bsuch as\b',
        r'\btherefore\b', r'\bthus\b', r'\bas a result\b', r'\bconsequently\b',
        r'\bfirst\b', r'\bsecond\b', r'\bthird\b', r'\bfinally\b',
        r'\bimportant\b', r'\bkey\b', r'\bcrucial\b', r'\bessential\b'
    ]

    educational_score = sum(
        1 for indicator in educational_indicators
        if re.search(indicator, content, re.IGNORECASE)
    )

    return {
        "valid": True,
        "educational_score": educational_score,
        "message": f"Educational content validated with score: {educational_score}"
    }