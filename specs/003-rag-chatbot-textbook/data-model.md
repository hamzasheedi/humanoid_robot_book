# Data Model: RAG Chatbot Integration

## Entity: Question
- **id**: UUID (primary key)
- **content**: String (the text of the question)
- **source_context**: String (selected text or page context)
- **user_id**: UUID (foreign key to user, optional for anonymous usage)
- **session_id**: UUID (foreign key to chat session)
- **timestamp**: DateTime (when the question was asked)
- **metadata**: JSON (additional context like page location, user agent)

## Entity: Answer
- **id**: UUID (primary key)
- **question_id**: UUID (foreign key to Question)
- **content**: String (the text of the answer)
- **sources**: Array[String] (list of textbook sections cited)
- **confidence_score**: Float (0.0 to 1.0, measure of answer accuracy)
- **timestamp**: DateTime (when the answer was generated)
- **metadata**: JSON (additional context like LLM used, tokens consumed)

## Entity: Chat Session
- **id**: UUID (primary key)
- **user_id**: UUID (foreign key to user, optional)
- **session_start**: DateTime
- **session_end**: DateTime (nullable, null if active)
- **interaction_count**: Integer (number of question-answer pairs in session)
- **metadata**: JSON (session-specific settings, preferences)

## Entity: Textbook Content
- **id**: UUID (primary key)
- **title**: String (title of the content chunk)
- **content**: Text (the actual text content for RAG context)
- **module**: String (e.g., "ROS 2", "Gazebo/Unity", "NVIDIA Isaac", "VLA", "Capstone")
- **section**: String (specific section within the module)
- **page_reference**: String (link or reference to the original textbook page)
- **embedding_id**: String (ID in Qdrant vector store)
- **created_at**: DateTime
- **updated_at**: DateTime

## Entity: User Interaction Log
- **id**: UUID (primary key)
- **question_id**: UUID (foreign key to Question)
- **answer_id**: UUID (foreign key to Answer)
- **accuracy_rating**: Integer (nullable, 1-5 scale if user rated)
- **useful**: Boolean (whether user found the answer useful)
- **feedback_text**: Text (optional user feedback)
- **timestamp**: DateTime
- **metadata**: JSON (additional analytics data)

## Relationships:
- A Chat Session contains many Question-Answer pairs
- A Question links to one Answer
- A Question is part of one Chat Session
- An Answer references multiple Textbook Content chunks as sources
- A User Interaction Log connects a Question and Answer with additional analytics